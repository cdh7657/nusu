// 누수감지기 v2 모의 시험: 집 배관 모형(밸브 열림/잠김, 수전, 누수)으로 core.h 판정을 돌려 본다.
// 모형 값(공급압·배관 탄성·수전 유량)은 가정이다. 실제 주택 성능으로 보고하지 않는다.
#include "../firmware/nusu_leak2/core.h"
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <string>

struct Use { double tStart; double dur; double q; double rampMs; };  // 초, 초, L/분, 열림 시간(ms)

struct House {
  double Ps = 3.5;      // 공급압(bar)
  double k = 0.003;     // 열린 상태 마찰: p = Ps - k q^2
  double C = 0.10;      // 닫힌 배관 탄성(L/bar)
  double p = 3.5;
  bool valve = true;
  double hammer = 0;    // 닫힘 수격(감쇠)
};

struct Result { int alerts[AL_COUNT] = {0}; double first[AL_COUNT]; int closes = 0, opens = 0; double lostL = 0; };

Result run(const char* name, std::vector<Use> uses, double leakStart, double leakQ, double hours, bool away = false,
           double burstAt = -1, double burstQ = 0, bool quiet = false) {
  House h; Core core; SigDetector sd; std::mt19937 rng(3); std::normal_distribution<double> n(0, 0.003);
  Result r; for (int i = 0; i < AL_COUNT; i++) r.first[i] = -1;
  const uint32_t T0 = 1767000000; // 임의 시작 시각(자정 기준 가정)
  double accP = 0; int accN = 0; double qSec = 0; int qN = 0;
  double prevTap = 0;
  for (long ms = 0; ms < (long)(hours * 3600000); ms++) {
    double ts = ms / 1000.0;
    // 수전 요구 유량(밸브 열려 있다고 볼 때)
    double tap = 0;
    for (auto& u : uses) {
      if (ts >= u.tStart && ts < u.tStart + u.dur) {
        double e = (ts - u.tStart) * 1000.0; double f = e < u.rampMs ? e / u.rampMs : 1.0;
        double eEnd = (u.tStart + u.dur - ts) * 1000.0; if (eEnd < 80) f *= eEnd / 80.0;  // 닫힘 80ms
        tap += u.q * f;
      }
    }
    double leak = ts >= leakStart && leakStart >= 0 ? leakQ : 0;
    if (burstAt >= 0 && ts >= burstAt) leak += burstQ;
    double qOut, qMeter;
    if (h.valve) {
      qOut = tap + leak;
      double target = h.Ps - h.k * qOut * qOut;
      if (prevTap > tap + 1.0) h.hammer += 0.15;           // 수전 닫힘 → 수격
      h.hammer *= 0.995;
      h.p += (target + h.hammer * std::sin(ms * 0.3) - h.p) * 0.2;
      qMeter = qOut;
    } else {
      // 닫힌 배관: 압력에 비례해 수전이 빼가는 양(오리피스) + 누수
      double frac = h.p > 0 ? std::sqrt(std::max(0.0, h.p) / h.Ps) : 0;
      qOut = (tap + leak) * frac;
      h.p -= (qOut / 60.0 / 1000.0) / h.C;   // L/ms → bar
      if (h.p < 0) h.p = 0;
      qMeter = 0;
    }
    prevTap = tap;
    r.lostL += leak * (h.valve ? 1.0 : std::sqrt(std::max(0.0, h.p) / h.Ps)) / 60000.0;
    double pm = h.p + n(rng);
    sd.openThr = sd.closeThr = core.valveOpen ? core.c.openThrOpen : core.c.openThrClosed;
    sd.feed((uint32_t)ms, (float)pm);
    accP += pm; accN++; qSec += qMeter; qN++;
    if (ms % 1000 == 999) {
      Inputs in; in.t = T0 + (uint32_t)(ts); in.hour = (uint8_t)(((long)ts / 3600) % 24);
      in.q = (float)(qSec / qN + std::normal_distribution<double>(0, 0.01)(rng)); if (in.q < 0) in.q = 0;
      in.p = (float)(accP / accN); in.away = away; in.trickle = false; in.trickleLpm = 0; in.cmdOpen = false; in.cmdTest = false; in.ack = false; in.cmdClose = false;
      Out o = core.step(in, sd.out, sd.nout); sd.nout = 0;
      if (o.act == ACT_CLOSE) { h.valve = false; r.closes++; }
      if (o.act == ACT_OPEN) { h.valve = true; r.opens++; }
      for (int i = 0; i < o.na; i++) { r.alerts[o.alerts[i]]++; if (r.first[o.alerts[i]] < 0) r.first[o.alerts[i]] = ts; }
      accP = 0; accN = 0; qSec = 0; qN = 0;
    }
  }
  if (!quiet) {
    printf("%-44s", name);
    bool any = false;
    for (int a = 1; a < AL_COUNT; a++) if (r.alerts[a]) {
      any = true; printf(" %s x%d", ALERT_NAME[a], r.alerts[a]);
      if (leakStart >= 0 && r.first[a] >= leakStart) printf("(누수 후 %.0f분)", (r.first[a] - leakStart) / 60.0);
      else if (burstAt >= 0 && r.first[a] >= burstAt) printf("(터짐 후 %.1f분)", (r.first[a] - burstAt) / 60.0);
      else printf("(%02d:%02d)", (int)(r.first[a] / 3600) % 24, (int)(r.first[a] / 60) % 60);
    }
    if (!any) printf(" 알림 없음");
    printf(" | 밸브 잠금 %d·열림 %d", r.closes, r.opens);
    if (leakStart >= 0 || burstAt >= 0) printf(" | 샌 물 %.1fL", r.lostL);
    printf("\n");
  }
  return r;
}

std::vector<Use> day(int days) {
  std::vector<Use> v;
  for (int d = 0; d < days; d++) {
    double o = d * 86400.0;
    v.push_back({o + 7 * 3600, 480, 9, 400});            // 아침 샤워
    for (int i = 0; i < 8; i++) v.push_back({o + 7.2 * 3600 + i * 5400, 45, 6, 250});   // 변기
    for (int i = 0; i < 15; i++) v.push_back({o + 7.5 * 3600 + i * 3000, 25, 4, 300});   // 세면대·싱크대
    v.push_back({o + 12 * 3600, 600, 6, 200});           // 세탁기 급수 1
    v.push_back({o + 12.4 * 3600, 300, 6, 200});         // 세탁기 급수 2
    v.push_back({o + 19 * 3600, 900, 7, 500});           // 설거지(길게)
    v.push_back({o + 21.5 * 3600, 600, 9, 400});         // 저녁 샤워
    v.push_back({o + 23 * 3600, 30, 4, 300});
  }
  return v;
}

int main() {
  auto d2 = day(2);
  printf("== 모형 가정: 공급압 3.5bar, 열린 상태 손실 0.003·q², 닫힌 배관 탄성 0.1L/bar, 수전 열림 0.2~0.5초 ==\n");
  run("정상 생활 2일", d2, -1, 0, 48);
  run("누수 분당 1L (낮 14:00 시작)", d2, 14 * 3600, 1.0, 48);
  run("누수 분당 1L (새벽 03:00 시작, 잠긴 동안)", d2, 27 * 3600, 1.0, 48);
  run("누수 분당 0.1L (처음부터)", d2, 0, 0.1, 30);
  run("누수 분당 0.02L (물방울, 처음부터)", d2, 0, 0.02, 30);
  run("배관 터짐 분당 20L (낮 15:00)", d2, -1, 0, 30, false, 15 * 3600, 20);
  run("배관 터짐 분당 20L (새벽 03:00, 잠긴 동안)", d2, -1, 0, 30, false, 27 * 3600, 20);
  // 집 비움: 사용 없음
  run("집 비움, 누수 없음", {}, -1, 0, 24, true);
  run("집 비움, 누수 분당 1L (10:00 시작)", {}, 10 * 3600, 1.0, 24, true);
  // 잠긴 상태에서 수전을 살살 연 경우(분당 2L, 열림 2초)
  std::vector<Use> gentle = {{3 * 3600 + 100, 60, 2, 2000}};
  run("새벽 잠금 중 수전을 살살 엶(분당 2L)", gentle, -1, 0, 5);
  auto bath = day(1); bath.push_back({17 * 3600, 540, 18, 400});
  run("낮 욕조 받기 분당 18L 9분(정상)", bath, -1, 0, 24);
  std::vector<Use> fast = {{3 * 3600 + 100, 60, 6, 300}};
  run("새벽 잠금 중 수전을 보통으로 엶(분당 6L)", fast, -1, 0, 5);
}
