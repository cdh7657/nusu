// 누수 판정 로직 (아두이노와 무관한 순수 C++ — PC에서 시뮬레이션으로 검사함: leak/sim/sim.cpp)
// 1초에 한 번 feed(시각[초], 압력[bar])를 부르면, 알림을 보낼 일이 생겼을 때 EV_ 값을 돌려준다.
//
// 감압밸브+압력탱크(정밀 보급형)에서 물을 안 쓰면 감압밸브가 닫혀 압력이 "정지 압력"에 머문다.
//  EV_DRIP  : 물을 안 쓰는데 천천히 내려갔다 감압밸브가 '딱' 채우기를 규칙적으로 반복(톱니) → 미세 누수
//  EV_DECAY : 물을 안 쓰는데 톱니 없이 30분 동안 계속 내려감                              → 미세 누수
//  EV_FLOW  : 60분 동안 한 번도 정지 압력으로 못 돌아옴(감압밸브가 계속 열려 흐름)          → 누수·터짐·열어둠
// 감압밸브가 없는 집(기본형)은 본관 압력이 몇 시간에 걸쳐 오르내리므로 정지 압력을 쓸 수 없다.
//  → 기본형 EV_FLOW: 압력이 '뚝' 떨어진 뒤(터짐·틀어둠) 60분 동안 떨어지기 전 압력으로 못 돌아오면 알림.
//    작은 누수는 기본형으로 못 잡는다 → 월 1회 잠금 테스트로 확인.
#pragma once
#include <math.h>
#include <stdint.h>

enum { EV_NONE = 0, EV_DRIP = 1, EV_DECAY = 2, EV_FLOW = 3 };

struct Detector {
  // ---- 기준값 (서버 /leak/{코드}/cfg 로 바꿀 수 있음) ----
  float sharpBar = 0.10f;    // 2초 안에 이만큼 떨어지면 "물을 틂(사용)"
  float riseBar = 0.06f;     // 3초 안에 이만큼 오르면 "감압밸브가 채움(톱니 한 번)"
  float decayBar = 0.30f;    // 30분 동안 이만큼 계속 내려가면 DECAY
  float flowBar = 0.08f;     // 정지 압력보다 이만큼 낮은 상태로
  uint32_t flowSec = 3600;   //   이 시간 동안 한 번도 못 돌아오면 FLOW
  uint32_t quietAfterUse = 30;
  uint32_t refract = 5;      // 톱니 한 번을 두 번 세지 않도록
  uint32_t cooldown = 6 * 3600;  // 같은 알림은 6시간에 한 번
  int dripMin = 5;           // 1시간 안 톱니 수
  float dripCv = 0.5f;       // 톱니 간격이 이만큼 규칙적이면(변동계수)

  bool basic = false;
  float stepBack = 0.10f;    // 기본형: 떨어지기 전 압력에서 이 안으로 돌아오면 회복
  void basicMode() { basic = true; }  // 감압밸브 없는 집
  void seedBase(float b) { if (b > 0) base = b; }  // 설치 때 잰 정지 압력

  // ---- 상태 ----
  float r[8]; int ri = 0, rn = 0;          // 최근 8초
  float base = NAN;                         // 정지 압력(감압밸브가 닫혔을 때 압력)
  uint32_t t0 = 0, lastSharp = 0, lastRefill = 0, lastHigh = 0;
  uint32_t ref[40]; int nref = 0;
  float mP[31]; uint32_t mT[31]; int mi = 0, mn = 0; uint32_t lastMin = 0;
  float minMax = -1;                        // 이번 1분 최고 압력
  uint32_t lastFire[4] = {0, 0, 0, 0};
  float lastPeriod = 0;                     // DRIP 톱니 평균 간격(초)
  bool pend = false; float pendP = 0; uint32_t pendT = 0;  // 기본형: 뚝 떨어진 뒤 회복 기다림

  float ago(int k) const { return r[(ri - k + 16) % 8]; }  // k초 전 (1~8)
  void pushRefill(uint32_t t) {
    if (nref == 40) { for (int i = 1; i < 40; i++) ref[i - 1] = ref[i]; nref = 39; }
    ref[nref++] = t;
  }
  bool fire(int ev, uint32_t t) {
    if (lastFire[ev] && t - lastFire[ev] < cooldown) return false;
    lastFire[ev] = t; return true;
  }
  bool quiet(uint32_t t) const { return !lastSharp || t - lastSharp > quietAfterUse; }

  int feed(uint32_t t, float p) {
    if (!t0) { t0 = t; lastHigh = t; }
    if (rn >= 2 && p < ago(2) - sharpBar) {
      lastSharp = t;
      if (!pend && rn >= 4) { pend = true; pendP = ago(4); pendT = t; }
    }
    if (pend && p >= pendP - stepBack) pend = false;
    bool q = quiet(t);
    if (rn >= 3) {
      float mn3 = fminf(ago(1), fminf(ago(2), ago(3)));
      if (q && p - mn3 >= riseBar && t - lastRefill > refract) { pushRefill(t); lastRefill = t; }
    }
    r[ri] = p; ri = (ri + 1) % 8; if (rn < 8) rn++;

    // 정지 압력 = 분당 최고 압력의 윗선. 올라갈 땐 빨리, 내려갈 땐 하루 단위로 아주 천천히
    if (p > minMax) minMax = p;
    if (!lastMin || t - lastMin >= 60) {
      if (lastMin && minMax > 0) {
        if (isnan(base)) { if (t - t0 >= 120) base = minMax; }
        else if (minMax > base) base += (minMax - base) * 0.5f;
        else base += (minMax - base) * (1.0f / 1440.0f);
      }
      lastMin = t; mP[mi] = p; mT[mi] = t; mi = (mi + 1) % 31; if (mn < 31) mn++;
      minMax = p;
    }
    if (isnan(base) || p >= base - flowBar) lastHigh = t;

    int ev = EV_NONE;
    // DRIP: 마지막 사용 이후 최근 1시간 톱니가 규칙적인가
    uint32_t ws = t > 3600 ? t - 3600 : 0;
    if (lastSharp && lastSharp + quietAfterUse > ws) ws = lastSharp + quietAfterUse;
    {
      uint32_t ts[40]; int n = 0;
      for (int i = 0; i < nref; i++) if (ref[i] >= ws) ts[n++] = ref[i];
      if (n >= dripMin) {
        double sum = 0, sq = 0; int m = n - 1;
        for (int i = 1; i < n; i++) { double d = (double)(ts[i] - ts[i - 1]); sum += d; sq += d * d; }
        double mean = sum / m, sd = sqrt(fmax(0.0, sq / m - mean * mean));
        if (mean > 0 && sd / mean < dripCv && mean < 900) { lastPeriod = (float)mean; if (fire(EV_DRIP, t)) ev = EV_DRIP; }
      }
    }
    // DECAY: 30분 동안 사용·톱니 없이 계속 내려감
    if (!ev && mn >= 31 && (!lastSharp || t - lastSharp > 1800) && (!lastRefill || t - lastRefill > 1800)) {
      int old = mi;
      if (t - mT[old] >= 1700 && mP[old] - p >= decayBar) { if (fire(EV_DECAY, t)) ev = EV_DECAY; }
    }
    // FLOW: 오랫동안 정지 압력으로 못 돌아옴
    if (!ev) {
      bool longLow = basic ? (pend && t - pendT >= flowSec) : (!isnan(base) && t - lastHigh >= flowSec);
      if (longLow && fire(EV_FLOW, t)) ev = EV_FLOW;
    }
    return ev;
  }
};
