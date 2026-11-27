// 감압밸브+압력탱크 집 모형으로 detect.h 판정을 시험
#include "../firmware/nusu_leak/detect.h"
#include <cstdio>
#include <random>
#include <vector>
#include <string>
#include <cmath>

struct House {
  // 감압밸브: 설정 Ps, 잠김폭 db. 탱크 2L, 선압 2.6bar(게이지)
  double Ps = 3.0, db = 0.25, Vt = 2.0, pre = 2.6, P = 3.0, mains = 4.0;
  bool prvOpen = false; bool hasPrv = true; double qMod = 1.5; long tick = 0;
  double waterIn(double p) { // 탱크 안 물 부피(L), 보일 법칙
    double pa = p + 1.0, pre_a = pre + 1.0; if (p <= pre) return 0; return Vt * (1 - pre_a / pa);
  }
  double pFrom(double w) { double pre_a = pre + 1.0; if (w <= 0) return pre; return pre_a / (1 - w / Vt) - 1.0; }
  // q: 사용+누수 유량(L/분), 1초 진행
  double step(double q, std::mt19937& rng) {
    std::normal_distribution<double> n(0, 0.004);
    if (!hasPrv) { // 감압밸브 없음: 본관에 늘 연결, 유량에 따른 손실만
      P = mains + 0.3 * sin(tick++ * 6.2832 / 86400.0) - 0.004 * q * q; return P + n(rng);
    }
    double w = waterIn(P) - q / 60.0;
    if (q >= qMod) { // 흐름이 크면 감압밸브가 계속 열린 채 조절: 정지 압력보다 offset+droop 낮음
      prvOpen = true; double target = Ps - 0.10 - 0.012 * q; P += (target - P) * 0.5; return P + n(rng);
    }
    if (!prvOpen && pFrom(w) < Ps - db) prvOpen = true;
    if (prvOpen) { w += 0.4; double pn = pFrom(w); if (pn >= Ps) { pn = Ps; prvOpen = false; } P = pn; }
    else P = pFrom(w);
    if (P < 0) P = 0;
    return P + n(rng);
  }
};

struct Use { int start, dur; double q; };

int LEAKAT=0;
int run(const char* name, double leak, std::vector<Use> uses, int hours, bool hasPrv = true, int burstAt = -1, double burstQ = 0) {
  House h; h.hasPrv = hasPrv; Detector d; if (!hasPrv) d.basicMode(); std::mt19937 rng(7);
  int first[4] = {-1, -1, -1, -1}, cnt[4] = {0, 0, 0, 0};
  uint32_t T0 = 1700000000;
  for (int s = 0; s < hours * 3600; s++) {
    double q = s >= LEAKAT ? leak : 0;
    for (auto& u : uses) if (s >= u.start && s < u.start + u.dur) q += u.q;
    if (burstAt >= 0 && s >= burstAt) q += burstQ;
    double p = h.step(q, rng);
    int ev = d.feed(T0 + s, (float)p);
    if (ev) { cnt[ev]++; if (first[ev] < 0) first[ev] = s; }
  }
  const char* nm[4] = {"", "DRIP", "DECAY", "FLOW"};
  printf("%-34s", name);
  for (int e = 1; e < 4; e++) { if (cnt[e]) printf(" %s x%d(첫 %d분)", nm[e], cnt[e], first[e] / 60); }
  if (!cnt[1] && !cnt[2] && !cnt[3]) printf(" 알림 없음");
  if (cnt[1]) printf("  톱니간격 %.0f초", d.lastPeriod);
  printf("\n");
  return cnt[1] + cnt[2] + cnt[3];
}

int main() {
  // 하루 사용 패턴: 아침·저녁 샤워, 설거지, 세탁, 변기 여러 번
  std::vector<Use> day;
  int t[] = {7 * 3600, 7 * 3600 + 900, 8 * 3600, 12 * 3600, 13 * 3600, 18 * 3600, 19 * 3600, 21 * 3600, 22 * 3600 + 600};
  double q[] = {9, 6, 4, 4, 6, 4, 12, 9, 6};
  int du[] = {420, 60, 300, 240, 60, 600, 1500, 420, 60};
  for (int i = 0; i < 9; i++) day.push_back({t[i], du[i], q[i]});
  std::vector<Use> day3;
  for (int k = 0; k < 3; k++) for (auto u : day) { u.start += k * 86400; day3.push_back(u); }

  printf("== 감압밸브+탱크 ==\n");
  run("누수 없음, 3일 생활", 0, day3, 72);
  run("미세 누수 0.05L/분", 0.05, day3, 72);
  run("미세 누수 0.2L/분 (변기 물샘)", 0.2, {}, 24);
  run("누수 1L/분", 1.0, day3, 72);
  LEAKAT=86400; run("누수 3L/분 (둘째날부터)", 3.0, day3, 72); run("누수 1L/분 (둘째날부터)", 1.0, day3, 72); LEAKAT=0;
  run("터짐 15L/분 (둘째날 02시)", 0, day3, 72, true, 86400 + 2 * 3600, 15);
  run("마당 물주기 50분 (정상 사용)", 0, {{10 * 3600, 3000, 10}}, 24);
  run("마당 물주기 70분 (오탐 예상)", 0, {{10 * 3600, 4200, 10}}, 24);
  run("누수 0.2L/분 + 낮 생활 (밤에 잡히나)", 0.2, day3, 72);
  printf("== 감압밸브 없음(기본형) ==\n");
  run("누수 없음, 3일 생활", 0, day3, 72, false);
  run("미세 누수 0.2L/분", 0.2, day3, 72, false);
  run("터짐 15L/분", 0, day3, 72, false, 86400 + 2 * 3600, 15);
  run("누수 3L/분", 3.0, day3, 72, false);
}
