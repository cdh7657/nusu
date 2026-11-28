// 누수감지기 v2 핵심 로직 (아두이노와 무관한 순수 C++ — leak/sim2 에서 모의 시험)
//
// 구성: [메인밸브]→[스트레이너]→[래칭 솔레노이드 밸브]→[초음파 유량계]→[압력센서]→ 집 안 분기
//
// 1) SigDetector : 압력 1,000회/초를 받아 "수전 열림/닫힘 파형"과 "급격한 계단형 하강(파손 의심)"을 찾는다
// 2) Core        : 1초마다 유량·압력·파형 이벤트를 받아
//                  - 밸브가 열려 있을 때: 장시간 흐름·설명 안 되는 흐름·큰 흐름·집 비움 중 사용 판정
//                  - 1시간 안 쓰면 밸브를 잠그고 3분 압력시험 → 유지되면 "새지 않음 확인" 상태로 대기
//                  - 잠긴 동안 열림 파형이 오면 즉시 열고, 파형 없이 압력이 빠지면 누수로 보고 잠근 채 알림
// 모든 기준값은 Cfg 로 바꿀 수 있고, 서버 /leak/{코드}/cfg 로 내려받는다. 기준값은 시험 전 초기값이다.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

// ---------------- 파형 이벤트 ----------------
enum SigType { SIG_NONE = 0, SIG_OPEN = 1, SIG_CLOSE = 2, SIG_STEP = 3 };
struct Sig { uint8_t type; uint32_t tMs; float mag; uint16_t durMs; };

struct SigDetector {
  // 기준값
  float openThr = 0.04f;    // 50ms 동안 이만큼 떨어지면 열림 후보 (bar)
  float closeThr = 0.04f;   // 50ms 동안 이만큼 오르면 닫힘 후보
  uint16_t stepMs = 15;     // 이 시간 안에 대부분 떨어지면 "계단형"(파손 의심)
  float stepMag = 0.30f;    //   그리고 이만큼 이상 떨어질 때
  // 상태
  static const int N = 64;  // 64ms 기록 (1kHz)
  float buf[N]; int bi = 0, bn = 0;
  int mode = 0;             // 0 대기, -1 하강 추적, +1 상승 추적
  uint32_t evStart = 0, quietSince = 0;
  float evRef = 0, evExt = 0;
  static const int TR = 1500; float tr[TR]; int trn = 0;  // 하강 중 압력 기록(최대 1.5초)
  Sig out[8]; int nout = 0;  // Core가 1초마다 가져감

  float ago(int k) const { return buf[(bi - 1 - k + N * 2) % N]; }
  void emit(uint8_t ty, uint32_t t, float mag, uint16_t d) { if (nout < 8) out[nout++] = {ty, t, mag, d}; }

  // 센서 잡음을 줄이려고 8ms 이동평균을 먼저 낸다(수전 파형은 수십~수백 ms라 모양은 그대로)
  static const int MA = 8; float ma[MA]; int mi = 0, mn = 0; float msum = 0;
  void feed(uint32_t tMs, float raw) {
    if (mn == MA) msum -= ma[mi]; else mn++;
    ma[mi] = raw; msum += raw; mi = (mi + 1) % MA;
    float p = msum / mn;
    buf[bi] = p; bi = (bi + 1) % N; if (bn < N) bn++;
    if (bn < 51) return;
    float d50 = p - ago(50);
    if (mode == 0) {
      if (d50 < -openThr) {
        mode = -1; evStart = tMs - 50; evRef = ago(50); evExt = p; quietSince = 0; trn = 0;
        for (int k = 50; k >= 1; k--) tr[trn++] = ago(k);   // 시작 전 50ms 포함
        tr[trn++] = p;
      }
      else if (d50 > closeThr) { mode = +1; evStart = tMs - 50; evRef = ago(50); evExt = p; quietSince = 0; }
      return;
    }
    if (mode == -1) {
      if (p < evExt) evExt = p;
      if (trn < TR) tr[trn++] = p;
      // 하강이 멈췄는지(50ms 변화가 작음) 100ms 동안 확인. 1.5초 넘게 이어지면 그 자리에서 끝냄
      if (fabsf(d50) < openThr / 3) { if (!quietSince) quietSince = tMs; } else quietSince = 0;
      if ((quietSince && tMs - quietSince >= 100) || trn >= TR) {
        float mag = evRef - evExt;
        // 10%→90% 하강에 걸린 시간 = 하강 속도의 모양 (사람이 연 수전은 느리고, 파손은 급함)
        float a = evRef - 0.1f * mag, b = evRef - 0.9f * mag; int i10 = -1, i90 = -1;
        for (int i = 0; i < trn; i++) { if (i10 < 0 && tr[i] <= a) i10 = i; if (i90 < 0 && tr[i] <= b) { i90 = i; break; } }
        uint16_t dur = (i10 >= 0 && i90 >= i10) ? (uint16_t)(i90 - i10) : (uint16_t)trn;
        bool step = dur <= stepMs && mag >= stepMag;
        emit(step ? SIG_STEP : SIG_OPEN, evStart, mag, dur);
        mode = 0;
      }
    } else {
      if (p > evExt) evExt = p;
      if (fabsf(d50) < closeThr / 3) { if (!quietSince) quietSince = tMs; } else quietSince = 0;
      if (quietSince && tMs - quietSince >= 100) { emit(SIG_CLOSE, evStart, evExt - evRef, (uint16_t)(tMs - evStart)); mode = 0; }
    }
  }
};

// ---------------- 판정 ----------------
enum State { ST_OPEN = 0, ST_HOLD = 1, ST_CLOSED_OK = 2, ST_LEAK_HELD = 3, ST_OVERRIDE = 4 };
enum Alert {
  AL_NONE = 0, AL_BURST, AL_UNEXPLAINED, AL_RESIDUAL, AL_CONTINUOUS, AL_AWAY_USE,
  AL_HOLD_LEAK, AL_CLOSED_DROP, AL_VALVE_FAULT, AL_NIGHT_FLOW, AL_SENSOR, AL_PROTECT, AL_COUNT
};
static const char* ALERT_NAME[AL_COUNT] = {"", "burst", "unexplained", "residual", "continuous", "away_use",
                                           "hold_leak", "closed_drop", "valve_fault", "night_flow", "sensor", "protect"};
enum Action { ACT_NONE = 0, ACT_OPEN = 1, ACT_CLOSE = 2 };

struct Cfg {
  float qMin = 0.3f;          // 이 유량(L/분) 넘으면 "흐름 있음"
  float burstLpm = 15.0f;     // 큰 흐름
  uint32_t burstSec = 180;
  uint32_t unexplSec = 300;   // 열림 파형 없이 시작된 흐름이 이만큼 이어지면
  uint32_t residualSec = 180; // 닫힘 파형 뒤 남은 작은 흐름이 이만큼 이어지면
  float residualMax = 3.0f;
  uint32_t contDay = 1200, contNight = 900; // 장시간 흐름(초)
  float awayL = 10.0f;        // 집 비움 중 누적
  uint32_t idleCloseSec = 3600, idleCloseAwaySec = 60; // 안 쓰면 잠그기까지
  uint32_t holdSec = 180;     // 잠근 직후 압력시험
  float holdDrop = 0.15f;     // 시험 중 이만큼 빠지면 누수
  float useDrop = 0.30f;      // 확인 상태에서 10분 안에 이만큼 빠지면 "파형 없는 하강"
  uint32_t overrideSec = 7200;// 고객이 연 뒤 자동 잠금 안 하는 시간
  float mnfLpm = 0.05f;       // 새벽 최소유량 기준
  uint8_t nightFrom = 0, nightTo = 6; // 취침 시간대(시)
  bool autoProtect = true;    // 누수 판정 시 밸브를 잠가 둘지
  uint32_t burstAckSec = 600;  // 낮에 큰 흐름 확인 질문 뒤 응답 없으면 잠그기까지(초)
  float bigLpm = 12.0f;       // 설명 안 되는 흐름이 이만큼 크면 확인 기다리지 않고 잠금
  uint32_t ackWaitDay = 1800, ackWaitNight = 600; // 확인 질문 뒤 응답 없으면 잠그기까지(초)
  float openThrOpen = 0.012f, openThrClosed = 0.04f; // 파형 기준(밸브 열림/잠김). 잠김 기준은 그 집 배관 탄성으로 설치 때 맞춤
};

struct Inputs {
  uint32_t t;      // 초 (epoch)
  uint8_t hour;    // 현지 시각(시)
  float q;         // L/분, <0 이면 유량계 오류
  float p;         // bar, 1초 평균. <0 이면 압력센서 오류
  bool away, trickle; float trickleLpm;
  bool cmdOpen;    // 고객앱 "열기"
  bool cmdTest;    // 고객앱 "지금 점검"
  bool ack;        // 고객앱 "쓰는 중이에요" (흐름이 멈출 때까지 유효)
  bool cmdClose;   // 고객앱 "아니요, 안 써요" → 바로 잠그고 압력시험
};

struct Out { uint8_t act; uint8_t alerts[6]; int na; float info; };

struct Core {
  Cfg c;
  uint8_t st = ST_OPEN;
  bool valveOpen = true;
  // 흐름 추적
  bool flowing = false; uint32_t flowStart = 0, lastFlow = 0, lastSig = 0, lastOpenSig = 0, lastCloseSig = 0;
  bool startHadSig = false; uint32_t burstSince = 0, residSince = 0;
  float awayVol = 0; bool awayPrev = false;
  // 잠금 시험
  uint32_t stSince = 0; float p0 = 0; float pRef10[10]; int r10 = 0; uint32_t lastMin = 0;
  float pSupply = NAN;        // 열린 상태의 평소 정지 압력
  uint32_t openedAt = 0; bool openedBySig = false;
  // 기타
  uint32_t lastFire[AL_COUNT] = {0};
  uint32_t badQ = 0, badP = 0;
  float nightMin = 1e9; int nightDays = 0; uint8_t lastHour = 255;
  uint32_t lastTestPass = 0;
  uint32_t askT = 0; bool acked = false;   // 확인 질문을 보낸 시각, 고객 응답

  bool fire(Out& o, uint8_t a, uint32_t t, uint32_t cool = 6 * 3600) {
    if (lastFire[a] && t - lastFire[a] < cool) return false;
    lastFire[a] = t; if (o.na < 6) o.alerts[o.na++] = a; return true;
  }
  void go(uint8_t s, uint32_t t) { st = s; stSince = t; }
  void doClose(Out& o, uint32_t t, float p) { o.act = ACT_CLOSE; valveOpen = false; p0 = p; for (int i = 0; i < 10; i++) pRef10[i] = p; }
  void doOpen(Out& o, uint32_t t, bool bySig) { o.act = ACT_OPEN; valveOpen = true; openedAt = t; openedBySig = bySig; }

  // 1초마다 호출. sigs: 지난 1초 동안 SigDetector 가 찾은 파형
  Out step(const Inputs& in, const Sig* sigs, int ns) {
    Out o = {ACT_NONE, {0}, 0, 0};
    uint32_t t = in.t;
    bool openSig = false, stepSig = false;
    for (int i = 0; i < ns; i++) {
      if (sigs[i].type == SIG_OPEN) { openSig = true; lastOpenSig = t; lastSig = t; }
      if (sigs[i].type == SIG_CLOSE) { lastCloseSig = t; lastSig = t; }
      if (sigs[i].type == SIG_STEP) { stepSig = true; lastSig = t; }
    }
    // 센서 이상
    if (in.q < 0) { if (!badQ) badQ = t; } else badQ = 0;
    if (in.p < 0) { if (!badP) badP = t; } else badP = 0;
    if ((badQ && t - badQ > 300) || (badP && t - badP > 300)) fire(o, AL_SENSOR, t, 24 * 3600);

    float q = in.q < 0 ? 0 : in.q;
    float qEff = in.trickle ? fmaxf(0, q - in.trickleLpm) : q;
    bool night = c.nightFrom <= c.nightTo ? (in.hour >= c.nightFrom && in.hour < c.nightTo)
                                          : (in.hour >= c.nightFrom || in.hour < c.nightTo);
    // 10분 압력 기록(1분 간격)
    if (!lastMin || t - lastMin >= 60) { lastMin = t; pRef10[r10] = in.p; r10 = (r10 + 1) % 10; }
    float pMax10 = in.p; for (int i = 0; i < 10; i++) if (pRef10[i] > pMax10) pMax10 = pRef10[i];

    // 집 비움 누적
    if (in.away && !awayPrev) awayVol = 0;
    awayPrev = in.away;

    // 고객 "열기"
    if (in.cmdOpen && (st != ST_OPEN && st != ST_OVERRIDE)) { doOpen(o, t, false); go(ST_OVERRIDE, t); return o; }
    if (in.cmdOpen && st == ST_OPEN) { go(ST_OVERRIDE, t); }
    if (in.cmdClose && valveOpen) { doClose(o, t, in.p); go(ST_HOLD, t); askT = 0; return o; }

    switch (st) {
      case ST_OPEN:
      case ST_OVERRIDE: {
        if (st == ST_OVERRIDE && t - stSince >= c.overrideSec) go(ST_OPEN, t);
        // 열린 뒤 압력 회복 확인(파형으로 연 경우)
        if (openedBySig && openedAt && t - openedAt == 5 && !isnan(pSupply) && in.p < pSupply - 0.5f && qEff < c.qMin)
          fire(o, AL_VALVE_FAULT, t, 3600);
        // 평소 정지 압력 학습
        if (qEff < c.qMin && t - lastSig > 30) pSupply = isnan(pSupply) ? in.p : pSupply + (in.p - pSupply) * 0.01f;
        bool now = qEff >= c.qMin;
        if (now && !flowing) { flowStart = t; startHadSig = (t - lastOpenSig <= 3) || (lastSig && t - lastSig <= 3); residSince = 0; }
        if (now) lastFlow = t;
        flowing = now || (t - lastFlow < 10);  // 10초 이내 끊김은 이어진 것으로
        if (openSig && flowing) startHadSig = true;  // 사용 중 다른 수전 열림
        if (!flowing) { askT = 0; acked = false; }
        if (in.ack) acked = true;
        if (in.away) awayVol += qEff / 60.0f;

        // 큰 흐름
        if (qEff >= c.burstLpm) { if (!burstSince) burstSince = t; } else burstSince = 0;
        // 계단형 파형 = 바로 잠금. 큰 흐름이 이어지면 낮엔 확인 질문 뒤, 밤·집 비움엔 바로 잠금
        if (stepSig && fire(o, AL_BURST, t) && c.autoProtect && st != ST_OVERRIDE) { doClose(o, t, in.p); go(ST_LEAK_HELD, t); return o; }
        if (burstSince && t - burstSince >= c.burstSec && !acked) {
          bool f = fire(o, AL_BURST, t, 3600);
          if (f && !askT) askT = t;
          bool now2 = night || in.away || (askT && t - askT >= c.burstAckSec);
          if (now2 && c.autoProtect && st != ST_OVERRIDE) { fire(o, AL_PROTECT, t, 600); doClose(o, t, in.p); go(ST_LEAK_HELD, t); askT = 0; return o; }
        }
        // 집 비움 중 사용
        if (in.away && awayVol >= c.awayL) {
          if (fire(o, AL_AWAY_USE, t)) { if (c.autoProtect && st != ST_OVERRIDE) { doClose(o, t, in.p); go(ST_LEAK_HELD, t); return o; } }
        }
        if (flowing) {
          uint32_t dur = t - flowStart;
          // 열림 파형 없이 시작된 흐름
          if (!startHadSig && dur >= c.unexplSec) {
            bool f = fire(o, AL_UNEXPLAINED, t, 3600);
            if (f && !askT) askT = t;
            if (f && (in.away || qEff >= c.bigLpm) && c.autoProtect && st != ST_OVERRIDE) { doClose(o, t, in.p); go(ST_LEAK_HELD, t); return o; }
          }
          // 닫힘 파형 뒤 남은 작은 흐름
          if (lastCloseSig > lastOpenSig && lastCloseSig >= flowStart && qEff < c.residualMax) {
            if (!residSince) residSince = lastCloseSig;
            if (t - residSince >= c.residualSec && fire(o, AL_RESIDUAL, t, 3600) && !askT) askT = t;
          } else residSince = 0;
          // 장시간 흐름
          if (dur >= (night ? c.contNight : c.contDay) && fire(o, AL_CONTINUOUS, t, 3600) && !askT) askT = t;
          // 확인 질문에 응답이 없으면 잠금
          if (askT && !acked && t - askT >= (night ? c.ackWaitNight : c.ackWaitDay) && c.autoProtect && st != ST_OVERRIDE) {
            fire(o, AL_PROTECT, t, 600); doClose(o, t, in.p); go(ST_LEAK_HELD, t); askT = 0; return o;
          }
        }
        // 새벽 최소유량 (밸브가 밤새 열려 있을 때)
        if (in.hour != lastHour) {
          if (lastHour == 5 && in.hour == 6) {
            if (nightMin < 1e8 && nightMin >= c.mnfLpm) nightDays++; else nightDays = 0;
            if (nightDays >= 3) fire(o, AL_NIGHT_FLOW, t, 24 * 3600);
            nightMin = 1e9;
          }
          lastHour = in.hour;
        }
        if (in.hour >= 2 && in.hour < 5 && in.q >= 0 && qEff < nightMin) nightMin = qEff;

        // 안 쓰면 잠그기 (또는 "지금 점검")
        uint32_t idleNeed = in.away ? c.idleCloseAwaySec : c.idleCloseSec;
        bool idle = !flowing && (t - lastFlow >= idleNeed || !lastFlow) && (t - lastSig >= idleNeed || !lastSig) && t - stSince >= idleNeed;
        if (st == ST_OPEN && !in.trickle && in.p > 0 && (idle || in.cmdTest) && !flowing) { doClose(o, t, in.p); go(ST_HOLD, t); }
        break;
      }
      case ST_HOLD: {
        if (openSig) { doOpen(o, t, true); go(ST_OPEN, t); break; }        // 누가 수전을 엶
        if (p0 - in.p >= c.holdDrop) {                                       // 아무도 안 여는데 압력이 빠짐
          o.info = p0 - in.p;
          fire(o, AL_HOLD_LEAK, t, 24 * 3600);
          if (c.autoProtect) go(ST_LEAK_HELD, t); else { doOpen(o, t, false); go(ST_OPEN, t); }
          break;
        }
        if (t - stSince >= c.holdSec) { lastTestPass = t; go(ST_CLOSED_OK, t); }
        break;
      }
      case ST_CLOSED_OK: {
        if (openSig || stepSig) { doOpen(o, t, true); go(ST_OPEN, t); break; }   // 사용 → 즉시 열기
        if (pMax10 - in.p >= c.useDrop) {                                       // 파형 없이 빠짐
          o.info = pMax10 - in.p;
          if (in.away) { fire(o, AL_CLOSED_DROP, t, 3600); if (c.autoProtect) { go(ST_LEAK_HELD, t); break; } }
          doOpen(o, t, true); go(ST_OPEN, t);   // 평소: 천천히 연 수전일 수 있으니 열고, 열린 뒤 판정(설명 안 되는 흐름)으로 확인
          startHadSig = false; flowStart = t; flowing = true; lastFlow = t;
        }
        break;
      }
      case ST_LEAK_HELD: {
        // 잠근 채 유지. 하루 1번 다시 알림
        if (t - stSince >= 24 * 3600) { stSince = t; lastFire[AL_HOLD_LEAK] = 0; fire(o, AL_HOLD_LEAK, t); }
        break;
      }
    }
    return o;
  }
};
