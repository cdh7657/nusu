/*
  누수감지기 펌웨어 LK-2.0  (ESP32-S3, 보일러실 인입부 설치형)
  ------------------------------------------------------------
  배관 순서: [메인밸브]→[스트레이너]→[래칭 솔레노이드 밸브]→[초음파 유량계 DN15]→[압력센서]→ 집 안 분기
            ※ 보일러 보충수 배관은 밸브 "앞"에서 따로 뺀다 (밸브가 잠겨도 보일러는 물을 받아야 동파가 안 남)

  하는 일
   1) 압력 1,000회/초 → core.h SigDetector 가 수전 열림/닫힘/계단형(파손) 파형을 찾음
   2) 1초마다 유량(RS485 Modbus) + 압력 + 파형 → core.h Core 가 판정, 밸브 열기/잠그기
   3) 알림은 /leak/{코드}/alerts 에 기록 → 서버(functions)가 고객·관리자 폰으로 푸시
   4) 1시간마다 기록(사용량·최대 유량·최저 압력·잠금 시간) → 서버가 주 1회 보고서
   5) 고객앱 명령: 열기 / 쓰는 중이에요 / 집 비움 / 물 틀어둠 / 지금 점검 / 수전 등록
   6) 와이파이 설정은 v1과 같음(BOOT 5초 → DAEWON-LEAK-XXXX)

  상태: 문법 확인과 모의 시험(leak/sim2)만 함. 실제 부품으로 시험하지 않음.
        ADS131M04 레지스터 값, 유량계 Modbus 레지스터는 부품 받은 뒤 데이터시트로 확인할 것.

  배선 (ESP32-S3 DevKitC 기준, 핀은 아래 상수로 바꿀 수 있음)
   ADS131M04  SCLK 12, DIN(MOSI) 11, DOUT(MISO) 13, CS 10, DRDY 9, SYNC/RESET 14, CLKIN ← 8.192MHz 발진기
              AIN0P/N ← 압력센서 출력(분압해서 ±1.2V 안으로)
   RS485(MAX3485) RO→RX 18, DI←TX 17, DE/RE 16      유량계 A/B
   밸브 H브리지(DRV8871) IN1 4, IN2 5   ※ 래칭 밸브: 펄스로만 움직이고 전기 끊겨도 그 자리에 있음
   정전 감지  5V 입력 분압 → GPIO 6 (ADC)   슈퍼캡(5.5V 1F)으로 정전 순간 "열림" 펄스 한 번
   LED 48(보드 RGB 대신 일반 LED 쓰면 바꿀 것)  버튼 0(BOOT)
*/
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SPI.h>
#include <time.h>
#include "core.h"

#define FW_VER "LK-2.0"
static const char* DB_URL = "https://daewon-booking-default-rtdb.asia-southeast1.firebasedatabase.app";
static const char* APP_URL = "https://cdh7657.github.io/nusu/?go=leak&dev=";

// ---- 핀 ----
static const int PIN_SCLK = 12, PIN_MOSI = 11, PIN_MISO = 13, PIN_CS = 10, PIN_DRDY = 9, PIN_SYNC = 14;
static const int PIN_RX = 18, PIN_TX = 17, PIN_DE = 16;
static const int PIN_V1 = 4, PIN_V2 = 5, PIN_VIN = 6;
static const int PIN_LED = 2, PIN_BTN = 0;

// ---- 보정값 (서버 cfg 로 바꿀 수 있음) ----
static float P_GAIN = 0.0f;     // bar / ADC 코드  (설치 때 0bar·기준압 두 점으로 맞춤, 0이면 아래 기본식)
static float P_OFF = 0.0f;
static float P_MAX = 10.0f;     // 센서 최대 압력(bar)
static float VIN_MIN = 4.3f;    // 5V 입력이 이 밑으로 떨어지면 정전으로 봄
static uint8_t MB_ID = 1;       // 유량계 Modbus 주소
static uint16_t MB_REG = 0;     // 순간유량 레지스터 (제품마다 다름 — 데이터시트로 확인)
static uint8_t MB_FMT = 0;      // 0: float ABCD, 1: float CDAB, 2: int32 × MB_SCALE
static float MB_SCALE = 0.001f;
static float MB_UNIT = 1.0f / 60.0f * 1000.0f;  // 유량계 단위 → L/분 (m³/h → ×1000/60)
static uint32_t MB_BAUD = 9600;
static uint16_t VALVE_PULSE_MS = 60;

Preferences nv;
WebServer web(80);
DNSServer dns;
Core core;
SigDetector sd;

String code, wSsid, wPass;
bool portal = false; uint32_t portalUntil = 0;

// ---------------- 압력 1kHz (ADS131M04) ----------------
SPIClass spiAdc(FSPI);
volatile float pFast = 0;            // 최근 압력(bar)
volatile double pAcc = 0; volatile uint32_t pAccN = 0;
static const int SNIP = 300;         // 알림 때 올릴 파형: 최근 3초, 100Hz
float snip[SNIP]; volatile int snipI = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t adcTask = nullptr;
Sig sigQ[8]; volatile int sigN = 0;   // 압력 작업 → loop 로 넘기는 파형
volatile bool adcOk = false;

uint16_t adcXfer16(uint16_t cmd) {  // 24비트 워드 6개 프레임: 첫 워드에 명령, 응답은 다음 프레임 첫 워드
  digitalWrite(PIN_CS, LOW);
  uint32_t w0 = spiAdc.transfer16(cmd); spiAdc.transfer(0);
  for (int i = 0; i < 5; i++) { spiAdc.transfer16(0); spiAdc.transfer(0); }
  digitalWrite(PIN_CS, HIGH);
  return (uint16_t)w0;
}
void adcWreg(uint8_t addr, uint16_t val) {  // WREG 1개: 011a aaaa a000 0000, 다음 워드에 값
  digitalWrite(PIN_CS, LOW);
  spiAdc.transfer16(0x6000 | ((uint16_t)addr << 7)); spiAdc.transfer(0);
  spiAdc.transfer16(val); spiAdc.transfer(0);
  for (int i = 0; i < 4; i++) { spiAdc.transfer16(0); spiAdc.transfer(0); }
  digitalWrite(PIN_CS, HIGH);
}
int32_t adcReadCh0() {
  digitalWrite(PIN_CS, LOW);
  spiAdc.transfer16(0); spiAdc.transfer(0);                       // 상태 워드
  uint32_t b2 = spiAdc.transfer(0), b1 = spiAdc.transfer(0), b0 = spiAdc.transfer(0);  // CH0
  for (int i = 0; i < 4; i++) { spiAdc.transfer16(0); spiAdc.transfer(0); }  // CH1~3, CRC
  digitalWrite(PIN_CS, HIGH);
  int32_t v = (int32_t)((b2 << 16) | (b1 << 8) | b0);
  if (v & 0x800000) v |= 0xFF000000;
  return v;
}
float codeToBar(int32_t c) {
  if (P_GAIN != 0) return c * P_GAIN + P_OFF;
  // 기본식: ±1.2V 풀스케일, 센서 0.5~4.5V를 1/4로 분압(0.125~1.125V) 가정
  float v = c / 8388608.0f * 1.2f * 4.0f;
  return (v - 0.5f) / 4.0f * P_MAX;
}

void IRAM_ATTR onDrdy() { BaseType_t w = pdFALSE; if (adcTask) vTaskNotifyGiveFromISR(adcTask, &w); if (w) portYIELD_FROM_ISR(); }

void adcLoop(void*) {
  uint32_t n = 0; float dec = 0; int decN = 0;
  for (;;) {
    if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20))) { adcOk = false; continue; }
    int32_t c = adcReadCh0();
    float p = codeToBar(c);
    adcOk = true;
    portENTER_CRITICAL(&mux); pFast = p; pAcc += p; pAccN++; portEXIT_CRITICAL(&mux);
    sd.feed(millis(), p);           // 파형 찾기
    if (sd.nout) {                  // 찾은 파형을 loop 쪽 대기열로 옮김
      portENTER_CRITICAL(&mux);
      for (int i = 0; i < sd.nout && sigN < 8; i++) sigQ[sigN++] = sd.out[i];
      portEXIT_CRITICAL(&mux);
      sd.nout = 0;
    }
    dec += p; if (++decN == 10) { snip[snipI] = dec / 10; snipI = (snipI + 1) % SNIP; dec = 0; decN = 0; }
    n++;
  }
}

void adcBegin() {
  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_SYNC, OUTPUT); digitalWrite(PIN_SYNC, LOW); delay(2); digitalWrite(PIN_SYNC, HIGH); delay(5);  // 리셋
  spiAdc.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);
  spiAdc.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  // CLOCK(0x03): CH0만 켬, OSR 4096(→ 8.192MHz에서 1,000회/초), 고분해능 모드  ※ 데이터시트로 확인
  adcWreg(0x03, (0x1 << 8) | (5 << 2) | 0x2);
  // GAIN1(0x04): CH0 이득 1
  adcWreg(0x04, 0x0000);
  pinMode(PIN_DRDY, INPUT_PULLUP);
  xTaskCreatePinnedToCore(adcLoop, "adc", 4096, nullptr, 5, &adcTask, 0);
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), onDrdy, FALLING);
}

float takeP() {  // 1초 평균
  portENTER_CRITICAL(&mux); double a = pAcc; uint32_t n = pAccN; pAcc = 0; pAccN = 0; portEXIT_CRITICAL(&mux);
  return (n > 200 && adcOk) ? (float)(a / n) : -1;
}

// ---------------- 유량계 (RS485 Modbus RTU, 기능코드 03) ----------------
HardwareSerial mb(1);
uint16_t crc16(const uint8_t* b, int n) {
  uint16_t c = 0xFFFF;
  for (int i = 0; i < n; i++) { c ^= b[i]; for (int k = 0; k < 8; k++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1; }
  return c;
}
float readFlow() {  // L/분, 실패 -1
  uint8_t q[8] = {MB_ID, 0x03, (uint8_t)(MB_REG >> 8), (uint8_t)MB_REG, 0, 2, 0, 0};
  uint16_t c = crc16(q, 6); q[6] = c & 0xFF; q[7] = c >> 8;
  while (mb.available()) mb.read();
  digitalWrite(PIN_DE, HIGH); mb.write(q, 8); mb.flush(); digitalWrite(PIN_DE, LOW);
  uint8_t r[9]; int n = 0; uint32_t st = millis();
  while (n < 9 && millis() - st < 120) { if (mb.available()) r[n++] = mb.read(); }
  if (n < 9 || r[0] != MB_ID || r[1] != 0x03 || r[2] != 4) return -1;
  if (crc16(r, 7) != (uint16_t)(r[7] | (r[8] << 8))) return -1;
  uint32_t u = MB_FMT == 1 ? ((uint32_t)r[5] << 24 | (uint32_t)r[6] << 16 | (uint32_t)r[3] << 8 | r[4])
                           : ((uint32_t)r[3] << 24 | (uint32_t)r[4] << 16 | (uint32_t)r[5] << 8 | r[6]);
  float v; if (MB_FMT == 2) v = (int32_t)u * MB_SCALE; else memcpy(&v, &u, 4);
  v *= MB_UNIT;
  return (v < 0 || v > 200 || isnan(v)) ? -1 : v;
}

// ---------------- 밸브 ----------------
bool valveIsOpen = true;
void valvePulse(bool open) {
  digitalWrite(PIN_V1, open ? HIGH : LOW); digitalWrite(PIN_V2, open ? LOW : HIGH);
  delay(VALVE_PULSE_MS);
  digitalWrite(PIN_V1, LOW); digitalWrite(PIN_V2, LOW);
  valveIsOpen = open; core.valveOpen = open;
  nv.putBool("vopen", open);
}
float readVin() { uint32_t mv = 0; for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VIN); return mv / 8.0f / 1000.0f * 2.0f; }

// ---------------- 공통 ----------------
uint32_t nowEpoch() { time_t t = time(nullptr); return t > 1700000000 ? (uint32_t)t : 0; }
String makeCode() { const char* A = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789"; String c; for (int i = 0; i < 10; i++) c += A[esp_random() % 32]; return c; }
String prettyCode() { return code.substring(0, 5) + "-" + code.substring(5); }
void led(bool on) { digitalWrite(PIN_LED, on ? HIGH : LOW); }
String num(float v, int d = 2) { return isnan(v) ? String("null") : String(v, d); }

int db(const char* method, const String& path, const String& body = "", String* out = nullptr) {
  if (WiFi.status() != WL_CONNECTED) return -1;
  WiFiClientSecure c; c.setInsecure();  // 시제품: 인증서 확인 생략 (양산 전 루트 인증서 넣기)
  HTTPClient h; h.setTimeout(8000);
  if (!h.begin(c, String(DB_URL) + path + ".json")) return -2;
  h.addHeader("Content-Type", "application/json");
  int r = (strcmp(method, "GET") == 0) ? h.GET() : h.sendRequest(method, body);
  if (out && r > 0) *out = h.getString();
  h.end(); return r;
}
double jnum(const String& js, const char* key, double def) {
  int i = js.indexOf(String("\"") + key + "\""); if (i < 0) return def;
  i = js.indexOf(':', i); if (i < 0) return def;
  String v = js.substring(i + 1); v.trim(); if (v.startsWith("null")) return def;
  if (v.startsWith("true")) return 1;
  if (v.startsWith("false")) return 0;
  return v.toDouble();
}
String jstr(const String& js, const char* key) {
  int i = js.indexOf(String("\"") + key + "\""); if (i < 0) return "";
  i = js.indexOf(':', i); int a = js.indexOf('"', i + 1); int b = js.indexOf('"', a + 1);
  return (a < 0 || b < 0) ? "" : js.substring(a + 1, b);
}

// ---------------- 상태·명령 ----------------
struct { bool away = false, trickle = false; float trickleLpm = 0; uint32_t openReq = 0, testReq = 0, ackReq = 0, closeReq = 0, learnUntil = 0; String learnName; } C;
uint32_t doneOpen = 0, doneTest = 0, doneAck = 0, doneClose = 0;
float qNow = 0, pNow = 0;

// 1시간 기록
struct Hour { uint32_t key = 0; float vol = 0, qMax = 0, pMin = 99, pMax = -1, qMinNight = 999; uint32_t closedSec = 0, n = 0; uint16_t opens = 0, sigs = 0; } H;

const char* stName(uint8_t s) { static const char* N[] = {"open", "hold", "closed_ok", "leak_held", "override"}; return s < 5 ? N[s] : "?"; }

void sendAlert(uint8_t a, uint32_t t, float info) {
  String s = "[";
  int i0 = snipI; for (int i = 0; i < SNIP; i++) { if (i) s += ","; s += String(snip[(i0 + i) % SNIP], 3); }
  s += "]";
  String b = String("{\"type\":\"") + ALERT_NAME[a] + "\",\"t\":" + t + ",\"p\":" + num(pNow) + ",\"q\":" + num(qNow) +
             ",\"st\":\"" + stName(core.st) + "\",\"valve\":\"" + (valveIsOpen ? "open" : "closed") + "\",\"info\":" + num(info, 3) +
             ",\"away\":" + (C.away ? "true" : "false") + ",\"wave\":" + s + "}";
  int r = db("POST", "/leak/" + code + "/alerts", b);
  Serial.printf("[알림] %s → %d\n", ALERT_NAME[a], r);
}

void sendStatus(uint32_t t) {
  String b = String("{\"t\":") + t + ",\"p\":" + num(pNow) + ",\"q\":" + num(qNow) + ",\"st\":\"" + stName(core.st) +
             "\",\"valve\":\"" + (valveIsOpen ? "open" : "closed") + "\",\"lastTest\":" + core.lastTestPass +
             ",\"away\":" + (C.away ? "true" : "false") + ",\"trickle\":" + (C.trickle ? "true" : "false") +
             ",\"rssi\":" + WiFi.RSSI() + ",\"fw\":\"" FW_VER "\",\"kind\":\"v2\"}";
  db("PUT", "/leak/" + code + "/st", b);
  db("PUT", "/leakIndex/" + code + "/seen", String(t));
}

void sendHour() {
  if (!H.n) return;
  String b = String("{\"vol\":") + num(H.vol, 1) + ",\"qMax\":" + num(H.qMax, 1) + ",\"pMin\":" + num(H.pMin) + ",\"pMax\":" + num(H.pMax) +
             ",\"closed\":" + H.closedSec + ",\"opens\":" + H.opens + ",\"sigs\":" + H.sigs +
             (H.qMinNight < 999 ? String(",\"qNight\":") + num(H.qMinNight, 3) : String("")) + "}";
  db("PUT", "/leak/" + code + "/hist/" + String(H.key), b);
}

void pullCfg() {
  String js; if (db("GET", "/leak/" + code + "/cfg", "", &js) != 200 || js == "null") return;
  Cfg& c = core.c;
  c.qMin = jnum(js, "qMin", c.qMin);
  c.burstLpm = jnum(js, "burstLpm", c.burstLpm); c.burstSec = jnum(js, "burstSec", c.burstSec);
  c.unexplSec = jnum(js, "unexplSec", c.unexplSec);
  c.contDay = jnum(js, "contDay", c.contDay); c.contNight = jnum(js, "contNight", c.contNight);
  c.awayL = jnum(js, "awayL", c.awayL);
  c.idleCloseSec = jnum(js, "idleCloseSec", c.idleCloseSec);
  c.holdSec = jnum(js, "holdSec", c.holdSec); c.holdDrop = jnum(js, "holdDrop", c.holdDrop); c.useDrop = jnum(js, "useDrop", c.useDrop);
  c.nightFrom = jnum(js, "nightFrom", c.nightFrom); c.nightTo = jnum(js, "nightTo", c.nightTo);
  c.autoProtect = jnum(js, "autoProtect", c.autoProtect) != 0;
  c.burstAckSec = jnum(js, "burstAckSec", c.burstAckSec); c.bigLpm = jnum(js, "bigLpm", c.bigLpm);
  c.ackWaitDay = jnum(js, "ackWaitDay", c.ackWaitDay); c.ackWaitNight = jnum(js, "ackWaitNight", c.ackWaitNight);
  c.openThrOpen = jnum(js, "openThrOpen", c.openThrOpen); c.openThrClosed = jnum(js, "openThrClosed", c.openThrClosed);
  P_GAIN = jnum(js, "pGain", P_GAIN); P_OFF = jnum(js, "pOff", P_OFF); P_MAX = jnum(js, "pmax", P_MAX);
  MB_ID = jnum(js, "mbId", MB_ID); MB_REG = jnum(js, "mbReg", MB_REG); MB_FMT = jnum(js, "mbFmt", MB_FMT);
  MB_SCALE = jnum(js, "mbScale", MB_SCALE); MB_UNIT = jnum(js, "mbUnit", MB_UNIT);
}

// 명령: { open: 시각, close: 시각, test: 시각, ack: 시각, away: true/false, trickle: L/분(0=끔), learn: {until, name} }
bool cmdFirst = true;
void pullCmd() {
  String js; if (db("GET", "/leak/" + code + "/cmd", "", &js) != 200 || js == "null") { cmdFirst = false; return; }
  C.openReq = jnum(js, "open", 0); C.testReq = jnum(js, "test", 0); C.ackReq = jnum(js, "ack", 0); C.closeReq = jnum(js, "close", 0);
  C.away = jnum(js, "away", 0) != 0;
  float tl = jnum(js, "trickle", 0); C.trickle = tl > 0; C.trickleLpm = tl;
  C.learnUntil = jnum(js, "until", 0); C.learnName = jstr(js, "name");
  if (cmdFirst) { cmdFirst = false; doneOpen = C.openReq; doneTest = C.testReq; doneAck = C.ackReq; doneClose = C.closeReq; }  // 켜기 전 명령은 무시
}

// 수전 등록: 등록 시간 동안 나온 파형과 5초 뒤 유량을 올림 → 서버/앱에서 이름 붙여 저장
struct PendFx { bool on = false; uint32_t t = 0; Sig s; } fx;
void sendFx(float q) {
  String b = String("{\"name\":\"") + C.learnName + "\",\"t\":" + fx.t + ",\"type\":" + fx.s.type + ",\"mag\":" + num(fx.s.mag, 3) +
             ",\"dur\":" + fx.s.durMs + ",\"q\":" + num(q, 2) + ",\"valve\":\"" + (valveIsOpen ? "open" : "closed") + "\"}";
  db("POST", "/leak/" + code + "/fx", b);
}

// ---------------- 와이파이 설정 화면 (v1과 같음) ----------------
const char PAGE_HEAD[] PROGMEM =
  "<!doctype html><html lang=ko><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>누수감지기 설정</title><style>body{font:18px/1.5 sans-serif;margin:0;background:#f7f0e1;color:#33261a}"
  ".w{max-width:480px;margin:0 auto;padding:18px}h1{font-size:22px}.c{background:#fff;border-radius:14px;padding:16px;margin:12px 0}"
  "select,input,button{font:inherit;width:100%;padding:13px;margin:6px 0;border-radius:12px;border:1.5px solid #ccb98f;box-sizing:border-box}"
  "button{background:#33261a;color:#fff;font-weight:700;border:0}.code{font-size:30px;font-weight:900;letter-spacing:3px;text-align:center}"
  ".s{color:#7a6a58;font-size:15px}</style></head><body><div class=w>";

void pageRoot() {
  int n = WiFi.scanNetworks();
  String h = FPSTR(PAGE_HEAD);
  h += "<h1>대원 누수감지기 설정</h1><div class=c><div class=s>장치 코드</div><div class=code>" + prettyCode() + "</div>"
       "<div class=s>압력 <b id=p>" + num(pNow) + "</b> bar · 유량 <b id=q>" + num(qNow, 1) + "</b> L/분</div></div>"
       "<form class=c method=post action=/save><b>집 와이파이</b><select name=s>";
  for (int i = 0; i < n; i++) { String s = WiFi.SSID(i); if (!s.length()) continue; h += "<option" + String(s == wSsid ? " selected" : "") + ">" + s + "</option>"; }
  h += "</select><input name=m placeholder='목록에 없으면 직접 입력'><input name=k type=password placeholder='와이파이 비밀번호'>"
       "<button>저장하고 연결</button></form>"
       "<p class=s>저장하면 장치가 집 와이파이에 연결돼요. 그다음 고객앱 <b>집 → 누수감지기 연결</b>에서 위 코드를 넣어 주세요.</p>"
       "<script>setInterval(function(){fetch('/p').then(function(r){return r.json()}).then(function(j){p.textContent=j.p;q.textContent=j.q})},2000)</script>"
       "</div></body></html>";
  web.send(200, "text/html; charset=utf-8", h);
}
void pageSave() {
  String s = web.arg("m").length() ? web.arg("m") : web.arg("s"), k = web.arg("k");
  String h = FPSTR(PAGE_HEAD);
  WiFi.begin(s.c_str(), k.c_str());
  uint32_t st = millis(); while (WiFi.status() != WL_CONNECTED && millis() - st < 15000) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    nv.putString("ssid", s); nv.putString("pass", k); wSsid = s; wPass = k;
    h += "<h1>연결됐어요 ✅</h1><div class=c><div class=code>" + prettyCode() + "</div></div><div class=c style='word-break:break-all'>" +
         String(APP_URL) + code + "</div><p class=s>10초 뒤 설정용 와이파이가 꺼져요.</p></body></html>";
    web.send(200, "text/html; charset=utf-8", h); portalUntil = millis() + 10000;
  } else {
    WiFi.disconnect();
    h += "<h1>연결이 안 됐어요</h1><p>와이파이 이름과 비밀번호를 다시 확인해 주세요.</p><a href=/><button>다시 하기</button></a></div></body></html>";
    web.send(200, "text/html; charset=utf-8", h);
  }
}
void startPortal(uint32_t minutes) {
  portal = true; portalUntil = millis() + minutes * 60000UL;
  WiFi.mode(WIFI_AP_STA);
  String ap = "DAEWON-LEAK-" + code.substring(6);
  WiFi.softAP(ap.c_str()); dns.start(53, "*", WiFi.softAPIP());
  web.on("/", pageRoot); web.on("/save", HTTP_POST, pageSave);
  web.on("/p", []() { web.send(200, "application/json", "{\"p\":" + num(pNow) + ",\"q\":" + num(qNow, 1) + "}"); });
  web.onNotFound([]() { web.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/"); web.send(302); });
  web.begin();
}
void stopPortal() { web.stop(); dns.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA); portal = false; }
bool joinWifi(uint32_t ms) {
  if (!wSsid.length()) return false;
  WiFi.mode(WIFI_STA); WiFi.begin(wSsid.c_str(), wPass.c_str());
  uint32_t st = millis(); while (WiFi.status() != WL_CONNECTED && millis() - st < ms) delay(250);
  return WiFi.status() == WL_CONNECTED;
}

// ---------------- 시작 ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT); led(false);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_V1, OUTPUT); pinMode(PIN_V2, OUTPUT); digitalWrite(PIN_V1, LOW); digitalWrite(PIN_V2, LOW);
  pinMode(PIN_DE, OUTPUT); digitalWrite(PIN_DE, LOW);
  analogReadResolution(12);

  nv.begin("leak2", false);
  code = nv.getString("code", "");
  if (code.length() != 10) { code = makeCode(); nv.putString("code", code); }
  wSsid = nv.getString("ssid", ""); wPass = nv.getString("pass", "");
  Serial.printf("\n대원 누수감지기 %s  코드 %s\n", FW_VER, prettyCode().c_str());

  // 켜질 때: 누수로 잠가 둔 상태였으면 그대로, 아니면 연다 (전원 문제로 물이 끊기지 않게)
  bool held = nv.getBool("held", false);
  if (held) { valvePulse(false); core.st = ST_LEAK_HELD; core.valveOpen = false; }
  else valvePulse(true);

  mb.begin(MB_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  adcBegin();
  if (!joinWifi(20000)) startPortal(10);
  configTime(9 * 3600, 0, "pool.ntp.org", "time.google.com");
}

uint32_t lastSec = 0, lastStatus = 0, lastCmd = 0, lastCfg = 0, btnDown = 0, lastRetry = 0;
bool booted = false, powerLost = false;

void loop() {
  uint32_t ms = millis();

  // 정전: 슈퍼캡 전기로 밸브를 열어 둠 (정전 중 물이 끊기지 않게). 누수로 잠근 상태면 그대로 둠
  if (!powerLost && readVin() < VIN_MIN) {
    powerLost = true;
    if (!valveIsOpen && core.st != ST_LEAK_HELD) valvePulse(true);
  } else if (powerLost && readVin() > VIN_MIN + 0.3f) powerLost = false;

  if (digitalRead(PIN_BTN) == LOW) { if (!btnDown) btnDown = ms; else if (ms - btnDown > 5000 && !portal) { btnDown = 0; startPortal(15); } }
  else btnDown = 0;
  if (portal) {
    dns.processNextRequest(); web.handleClient(); led((ms / 150) % 2);
    if ((int32_t)(ms - portalUntil) > 0) { stopPortal(); if (!joinWifi(20000) && !wSsid.length()) startPortal(10); }
  }

  // 1초마다 판정
  if (ms - lastSec >= 1000) {
    lastSec = ms;
    uint32_t t = nowEpoch();
    qNow = readFlow(); pNow = takeP();
    // 파형 기준은 밸브 상태에 따라 다름 (잠기면 배관이 단단해져 같은 수전도 압력 변화가 큼)
    sd.openThr = sd.closeThr = valveIsOpen ? core.c.openThrOpen : core.c.openThrClosed;
    Sig sigs[8]; int ns;
    portENTER_CRITICAL(&mux); ns = sigN; memcpy(sigs, sigQ, sizeof(Sig) * ns); sigN = 0; portEXIT_CRITICAL(&mux);
    if (t) {
      time_t tt = t; struct tm lt; localtime_r(&tt, &lt);
      Inputs in;
      in.t = t; in.hour = lt.tm_hour; in.q = qNow; in.p = pNow;
      in.away = C.away; in.trickle = C.trickle; in.trickleLpm = C.trickleLpm;
      in.cmdOpen = C.openReq && C.openReq != doneOpen; if (in.cmdOpen) doneOpen = C.openReq;
      in.cmdTest = C.testReq && C.testReq != doneTest; if (in.cmdTest) doneTest = C.testReq;
      in.ack = C.ackReq && C.ackReq != doneAck; if (in.ack) doneAck = C.ackReq;
      in.cmdClose = C.closeReq && C.closeReq != doneClose; if (in.cmdClose) doneClose = C.closeReq;
      uint8_t st0 = core.st;
      Out o = core.step(in, sigs, ns);
      if (o.act == ACT_CLOSE && valveIsOpen) valvePulse(false);
      if (o.act == ACT_OPEN && !valveIsOpen) { valvePulse(true); H.opens++; }
      if (core.st != st0) { nv.putBool("held", core.st == ST_LEAK_HELD); sendStatus(t); }
      for (int i = 0; i < o.na; i++) sendAlert(o.alerts[i], t, o.info);

      // 수전 등록
      if (C.learnUntil > t) {
        for (int i = 0; i < ns; i++) if (sigs[i].type == SIG_OPEN && !fx.on) { fx.on = true; fx.t = t; fx.s = sigs[i]; }
        if (fx.on && t - fx.t >= 5) { sendFx(qNow); fx.on = false; }
      } else fx.on = false;

      // 1시간 기록 (키 = YYYYMMDDHH)
      uint32_t key = (lt.tm_year + 1900) * 1000000UL + (lt.tm_mon + 1) * 10000UL + lt.tm_mday * 100UL + lt.tm_hour;
      if (H.key && key != H.key) { sendHour(); H = Hour(); }
      H.key = key; H.n++; H.sigs += ns;
      if (qNow > 0) { H.vol += qNow / 60.0f; if (qNow > H.qMax) H.qMax = qNow; }
      if (pNow >= 0) { if (pNow < H.pMin) H.pMin = pNow; if (pNow > H.pMax) H.pMax = pNow; }
      if (!valveIsOpen) H.closedSec++;
      if (valveIsOpen && lt.tm_hour >= 2 && lt.tm_hour < 5 && qNow >= 0 && qNow < H.qMinNight) H.qMinNight = qNow;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    uint32_t t = nowEpoch();
    if (t && !booted) { booted = true; pullCfg(); sendStatus(t); lastStatus = lastCfg = ms; }
    if (t && ms - lastStatus >= 600000UL) { lastStatus = ms; sendStatus(t); }
    if (ms - lastCfg >= 3600000UL) { lastCfg = ms; pullCfg(); }
    // 잠긴 동안·확인 질문 중엔 자주(5초), 평소엔 20초마다 명령 확인
    uint32_t every = (!valveIsOpen || core.askT || C.learnUntil > t) ? 5000UL : 20000UL;
    if (t && ms - lastCmd >= every) { lastCmd = ms; pullCmd(); }
    if (!portal) led(core.st == ST_LEAK_HELD ? ((ms / 300) % 2) : ((ms / 1000) % 10) == 0);
  } else if (!portal && ms - lastRetry > 60000UL) { lastRetry = ms; WiFi.reconnect(); }
}
