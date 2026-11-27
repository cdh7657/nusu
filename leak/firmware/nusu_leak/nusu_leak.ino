/*
  대원 누수알림기 펌웨어 LK-1.0  (ESP32-C3)
  ------------------------------------------------------------
  하는 일
   1) 1초마다 압력을 재서 detect.h로 누수를 판정
   2) 이상이 있으면 Firebase /leak/{장치코드}/alerts 에 기록 → 서버(functions)가 고객·관리자 폰으로 푸시
   3) 10분마다 상태, 1시간마다 기록(최저·최고·평균·톱니 수)을 올림 → 고객앱에서 보기, 1달 평균치 학습
   4) 고객앱에서 "잠금 테스트"를 누르면 2초마다 실시간 압력을 올림
   5) 와이파이 설정: 처음 켜거나 BOOT 버튼을 5초 누르면 "DAEWON-LEAK-XXXX" 와이파이가 생김
      → 휴대폰으로 접속하면 설정 화면이 자동으로 뜸 → 집 와이파이 고르고 비밀번호 입력

  배선 (ESP32-C3 SuperMini 기준)
   압력센서(5V, 0.5~4.5V 출력) 신호 → 10kΩ → GPIO3 → 20kΩ → GND  (4.5V가 3.0V로 낮아짐)
   센서 +5V → 5V핀, 센서 GND → GND
   LED: GPIO8(보드 내장, LOW=켜짐)   버튼: GPIO9(BOOT)

  아두이노 IDE 설정: 보드 "ESP32C3 Dev Module", USB CDC On Boot "Enabled"
*/
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include "detect.h"

#define FW_VER "LK-1.0"
static const char* DB_URL = "https://daewon-booking-default-rtdb.asia-southeast1.firebasedatabase.app";
static const char* APP_URL = "https://cdh7657.github.io/nusu/?go=leak&dev=";

// ---- 하드웨어 ----
static const int PIN_P = 3, PIN_LED = 8, PIN_BTN = 9;
static float P_MAX = 12.0f;        // 센서 최대 압력(bar). 1.2MPa 센서 = 12
static float DIV = 1.5f;           // 분압 비율(10k/20k → 1.5배)
static float ZERO_V = 0.50f;       // 0bar일 때 센서 전압

Preferences nv;
WebServer web(80);
DNSServer dns;
Detector det;

String code;            // 장치 코드(10자, 알림·연결 주소에 씀)
String wSsid, wPass;
bool portal = false;
uint32_t portalUntil = 0;
float pNow = 0;
uint32_t testUntil = 0;  // 잠금 테스트 끝 시각(epoch초)
uint32_t learnUntil = 0; // 1달 학습 끝 시각 — 그 전 미세누수 알림은 관리자에게만
bool alertOn = false;     // 알림 뒤 고객이 확인할 때까지 LED 켜 둠
uint32_t lastAlertT = 0;
uint32_t lastT = 0;

// 1시간 기록
struct { float mn = 99, mx = -1, sum = 0; uint32_t n = 0; int refills = 0, uses = 0; uint32_t hourKey = 0; } H;

// ---------------- 유틸 ----------------
uint32_t nowEpoch() { time_t t = time(nullptr); return t > 1700000000 ? (uint32_t)t : 0; }

String makeCode() {
  const char* A = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
  String c; for (int i = 0; i < 10; i++) c += A[esp_random() % 32]; return c;
}
String prettyCode() { return code.substring(0, 5) + "-" + code.substring(5); }

float readP() {
  uint32_t mv = 0; for (int i = 0; i < 32; i++) mv += analogReadMilliVolts(PIN_P);
  float v = (mv / 32.0f) / 1000.0f * DIV;
  float p = (v - ZERO_V) / 4.0f * P_MAX;
  return p < 0 ? 0 : p;
}

void led(bool on) { digitalWrite(PIN_LED, on ? LOW : HIGH); }

// Firebase 실시간 DB REST. path는 "/leak/..." 형식, ".json"은 자동으로 붙임
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

String num(float v, int d = 2) { return String(v, d); }

// JSON에서 숫자 하나 꺼내기 (작은 응답만 다루므로 간단히)
double jnum(const String& js, const char* key, double def) {
  int i = js.indexOf(String("\"") + key + "\""); if (i < 0) return def;
  i = js.indexOf(':', i); if (i < 0) return def;
  return js.substring(i + 1).toDouble();
}
String jstr(const String& js, const char* key) {
  int i = js.indexOf(String("\"") + key + "\""); if (i < 0) return "";
  i = js.indexOf(':', i); int a = js.indexOf('"', i + 1); int b = js.indexOf('"', a + 1);
  return (a < 0 || b < 0) ? "" : js.substring(a + 1, b);
}

// ---------------- 서버로 보내기 ----------------
void sendAlert(int ev, uint32_t t) {
  const char* type = ev == EV_DRIP ? "drip" : ev == EV_DECAY ? "decay" : "flow";
  bool learning = (ev != EV_FLOW) && learnUntil && t < learnUntil;
  String b = String("{\"type\":\"") + type + "\",\"t\":" + t + ",\"p\":" + num(pNow) +
             ",\"base\":" + (isnan(det.base) ? String("null") : num(det.base)) +
             ",\"period\":" + num(det.lastPeriod, 0) + ",\"learning\":" + (learning ? "true" : "false") + "}";
  int r = db("POST", "/leak/" + code + "/alerts", b);
  Serial.printf("[알림] %s → %d\n", type, r);
  alertOn = true; lastAlertT = t;
}

void sendStatus(uint32_t t) {
  String b = String("{\"t\":") + t + ",\"p\":" + num(pNow) + ",\"base\":" + (isnan(det.base) ? String("null") : num(det.base)) +
             ",\"rssi\":" + WiFi.RSSI() + ",\"fw\":\"" FW_VER "\",\"mode\":\"" + (det.basic ? "basic" : "prv") + "\"}";
  db("PUT", "/leak/" + code + "/st", b);
  db("PUT", "/leakIndex/" + code + "/seen", String(t));
}

void sendHour(uint32_t key) {
  if (!H.n) return;
  String b = String("{\"mn\":") + num(H.mn) + ",\"mx\":" + num(H.mx) + ",\"av\":" + num(H.sum / H.n) +
             ",\"rf\":" + H.refills + ",\"us\":" + H.uses + "}";
  db("PUT", "/leak/" + code + "/hist/" + String(key), b);
}

void pullCfg() {
  String js; if (db("GET", "/leak/" + code + "/cfg", "", &js) != 200 || js == "null") return;
  learnUntil = (uint32_t)jnum(js, "learnUntil", learnUntil);
  String mode = jstr(js, "mode"); if (mode == "basic") det.basicMode(); else if (mode == "prv") det.basic = false;
  det.riseBar = jnum(js, "riseBar", det.riseBar);
  det.flowBar = jnum(js, "flowBar", det.flowBar);
  det.flowSec = (uint32_t)jnum(js, "flowMin", det.flowSec / 60) * 60;
  float b = jnum(js, "base", 0); if (b > 0 && isnan(det.base)) det.seedBase(b);
  P_MAX = jnum(js, "pmax", P_MAX);
  ZERO_V = jnum(js, "zeroV", ZERO_V);
}

void pullCmd(uint32_t t) {
  String js; if (db("GET", "/leak/" + code + "/cmd", "", &js) != 200 || js == "null") return;
  uint32_t u = (uint32_t)jnum(js, "testUntil", 0);
  if (u > t && u - t <= 1800) { if (!testUntil) Serial.println("[잠금 테스트 시작]"); testUntil = u; }
  else if (testUntil && u == 0) testUntil = 1;  // 고객이 "그만하기" → 다음 차례에 끝냄
  if (jnum(js, "ack", 0) >= lastAlertT) alertOn = false;  // 고객이 알림을 확인함
}

// ---------------- 와이파이 설정 화면(휴대폰) ----------------
const char PAGE_HEAD[] PROGMEM =
  "<!doctype html><html lang=ko><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
  "<title>누수알림기 설정</title><style>body{font:18px/1.5 sans-serif;margin:0;background:#f7f0e1;color:#33261a}"
  ".w{max-width:480px;margin:0 auto;padding:18px}h1{font-size:22px}.c{background:#fff;border-radius:14px;padding:16px;margin:12px 0}"
  "select,input,button{font:inherit;width:100%;padding:13px;margin:6px 0;border-radius:12px;border:1.5px solid #ccb98f;box-sizing:border-box}"
  "button{background:#33261a;color:#fff;font-weight:700;border:0}.code{font-size:30px;font-weight:900;letter-spacing:3px;text-align:center}"
  ".s{color:#7a6a58;font-size:15px}</style></head><body><div class=w>";

void pageRoot() {
  int n = WiFi.scanNetworks();
  String h = FPSTR(PAGE_HEAD);
  h += "<h1>대원 누수알림기 설정</h1><div class=c><div class=s>장치 코드</div><div class=code>" + prettyCode() + "</div>"
       "<div class=s>지금 압력: <b id=p>" + num(pNow) + "</b> bar</div></div>"
       "<form class=c method=post action=/save><b>집 와이파이</b><select name=s>";
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i); if (!s.length()) continue;
    h += "<option" + String(s == wSsid ? " selected" : "") + ">" + s + "</option>";
  }
  h += "</select><input name=m placeholder='목록에 없으면 직접 입력'><input name=k type=password placeholder='와이파이 비밀번호'>"
       "<button>저장하고 연결</button></form>"
       "<p class=s>저장하면 장치가 집 와이파이에 연결되고 이 화면은 사라져요. 휴대폰이 원래 인터넷으로 돌아오면 "
       "고객앱에서 <b>집 → 누수알림기 연결</b>을 누르고 위 코드를 넣어 주세요.</p>"
       "<script>setInterval(function(){fetch('/p').then(function(r){return r.text()}).then(function(t){document.getElementById('p').textContent=t})},2000)</script>"
       "</div></body></html>";
  web.send(200, "text/html; charset=utf-8", h);
}

void pageSave() {
  String s = web.arg("m").length() ? web.arg("m") : web.arg("s");
  String k = web.arg("k");
  String h = FPSTR(PAGE_HEAD);
  WiFi.begin(s.c_str(), k.c_str());
  uint32_t st = millis(); while (WiFi.status() != WL_CONNECTED && millis() - st < 15000) delay(250);
  if (WiFi.status() == WL_CONNECTED) {
    nv.putString("ssid", s); nv.putString("pass", k); wSsid = s; wPass = k;
    h += "<h1>연결됐어요 ✅</h1><div class=c><div class=s>장치 코드</div><div class=code>" + prettyCode() + "</div></div>"
         "<p>이제 휴대폰을 원래 와이파이(또는 데이터)로 돌리고, 아래 주소를 열거나 고객앱에서 코드를 넣어 주세요.</p>"
         "<div class=c style='word-break:break-all'>" + String(APP_URL) + code + "</div><p class=s>10초 뒤 설정용 와이파이가 꺼져요.</p></body></html>";
    web.send(200, "text/html; charset=utf-8", h);
    portalUntil = millis() + 10000;
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
  WiFi.softAP(ap.c_str());
  dns.start(53, "*", WiFi.softAPIP());
  web.on("/", pageRoot);
  web.on("/save", HTTP_POST, pageSave);
  web.on("/p", []() { web.send(200, "text/plain", num(pNow)); });
  web.onNotFound([]() { web.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/"); web.send(302); });
  web.begin();
  Serial.printf("[설정 모드] 와이파이 %s 에 휴대폰으로 접속하세요\n", ap.c_str());
}

void stopPortal() {
  web.stop(); dns.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA); portal = false;
  Serial.println("[설정 모드 끝]");
}

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
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_P, ADC_11db);

  nv.begin("leak", false);
  code = nv.getString("code", "");
  if (code.length() != 10) { code = makeCode(); nv.putString("code", code); }
  wSsid = nv.getString("ssid", ""); wPass = nv.getString("pass", "");
  Serial.printf("\n대원 누수알림기 %s  코드 %s\n", FW_VER, prettyCode().c_str());

  pNow = readP();
  if (!joinWifi(20000)) startPortal(10);
  configTime(9 * 3600, 0, "pool.ntp.org", "time.google.com");
}

uint32_t lastSample = 0, lastStatus = 0, lastCmd = 0, lastCfg = 0, lastLive = 0, btnDown = 0, lastRetry = 0;
bool booted = false;

void loop() {
  uint32_t ms = millis();

  // BOOT 버튼 5초 → 와이파이 다시 설정
  if (digitalRead(PIN_BTN) == LOW) { if (!btnDown) btnDown = ms; else if (ms - btnDown > 5000 && !portal) { btnDown = 0; startPortal(15); } }
  else btnDown = 0;

  if (portal) {
    dns.processNextRequest(); web.handleClient();
    led((ms / 150) % 2);
    if ((int32_t)(ms - portalUntil) > 0) { stopPortal(); if (!joinWifi(20000) && wSsid.length() == 0) startPortal(10); }
  }

  // 1초마다 측정·판정
  if (ms - lastSample >= 1000) {
    lastSample = ms;
    pNow = readP();
    uint32_t t = nowEpoch();
    if (t) {
      if (lastT && t - lastT > 2) det.rn = 0;  // 통신으로 측정이 밀리면 최근 기록을 비움(급강하 오판 방지)
      lastT = t;
      uint32_t rf0 = det.lastRefill, sharp0 = det.lastSharp;
      int ev = det.feed(t, pNow);
      if (det.lastRefill != rf0) H.refills++;
      if (det.lastSharp != sharp0) H.uses++;
      // 1시간 기록 (키 = YYYYMMDDHH)
      time_t tt = t; struct tm lt; localtime_r(&tt, &lt);
      uint32_t key = (lt.tm_year + 1900) * 1000000UL + (lt.tm_mon + 1) * 10000UL + lt.tm_mday * 100UL + lt.tm_hour;
      if (H.hourKey && key != H.hourKey) { sendHour(H.hourKey); H = {}; }
      H.hourKey = key; H.n++; H.sum += pNow; if (pNow < H.mn) H.mn = pNow; if (pNow > H.mx) H.mx = pNow;
      if (ev) sendAlert(ev, t);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    uint32_t t = nowEpoch();
    if (t && !booted) { booted = true; pullCfg(); sendStatus(t); lastStatus = lastCfg = ms; }
    if (t && ms - lastStatus >= 600000UL) { lastStatus = ms; sendStatus(t); }
    if (ms - lastCfg >= 3600000UL) { lastCfg = ms; pullCfg(); }
    if (t && ms - lastCmd >= (testUntil ? 10000UL : 30000UL)) { lastCmd = ms; pullCmd(t); }
    if (t && testUntil) {
      if (t > testUntil) { testUntil = 0; db("DELETE", "/leak/" + code + "/live"); Serial.println("[잠금 테스트 끝]"); }
      else if (ms - lastLive >= 2000) { lastLive = ms; db("PUT", "/leak/" + code + "/live", String("{\"t\":") + t + ",\"p\":" + num(pNow, 3) + "}"); }
    }
    if (!portal) led(alertOn || ((ms / 1000) % 10) == 0);  // 정상: 10초에 한 번 깜빡, 알림 뒤엔 켜 둠
  } else if (!portal && ms - lastRetry > 60000UL) {
    lastRetry = ms; WiFi.reconnect();  // 끊기면 1분마다 다시 연결
  }
}
