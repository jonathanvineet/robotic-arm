// RoboArm — 6-DOF servo arm controller for ESP8266 / ESP32 + PCA9685.
//
//  * Serves the full web controller + 3D digital twin from flash (http://<ip>/)
//  * JSON commands over WebSocket (port 81), HTTP POST /api/cmd, or USB serial
//  * Smooth velocity/acceleration-limited motion, synced multi-joint moves
//  * Soft limits, home/reset, stop, E-stop, relax, calibration saved to flash
//  * Wi-Fi station with access-point fallback, mDNS, OTA updates
//
// Libraries (Arduino Library Manager):
//   Adafruit PWM Servo Driver Library, ArduinoJson (v7), WebSockets (Markus Sattler)

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
typedef ESP8266WebServer WebServerT;
#define BOARD_NAME "ESP8266"
#elif defined(ESP32)
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
typedef WebServer WebServerT;
#define BOARD_NAME "ESP32"
#else
#error "This sketch targets ESP8266 or ESP32"
#endif

#include "config.h"
#include "motion.h"
#include "web_assets.h"

// ============================================================ settings (flash)
static const uint32_t SETTINGS_MAGIC = 0x524F4241;  // "ROBA"
static const uint16_t SETTINGS_VERSION = 2;

struct JointCfg {
  float min, max, home, zero, vmax;
  int8_t dir;
  uint8_t ch;
  uint16_t pmin, pmax;
};

struct Settings {
  uint32_t magic;
  uint16_t version;
  float speed, accel;
  uint8_t sync, bootRelax;
  uint32_t osc;
  JointCfg j[NUM_JOINTS];
  char ssid[33];
  char pass[65];
};

Settings S;
motion::Planner<NUM_JOINTS> P;
Adafruit_PWMServoDriver pwm(PCA9685_ADDR, Wire);
WebServerT server(HTTP_PORT);
WebSocketsServer ws(WS_PORT);

bool estop = false;
bool relaxed = false;
bool pcaOk = false;
bool serialActive = false;  // a JSON client (web app over USB) is talking to us
bool staMode = false;
float ticksPerUs = 4096.0f * SERVO_FREQ / 1e6f;
uint16_t lastTicks[NUM_JOINTS];
String serialLine;

void loadDefaults() {
  memset(&S, 0, sizeof(S));
  S.magic = SETTINGS_MAGIC;
  S.version = SETTINGS_VERSION;
  S.speed = DEFAULT_SPEED;
  S.accel = DEFAULT_ACCEL;
  S.sync = DEFAULT_SYNC;
  S.bootRelax = DEFAULT_BOOT_RELAX;
  S.osc = PCA9685_OSC_DEFAULT;
  for (int i = 0; i < NUM_JOINTS; i++) {
    const JointDefaults& d = JOINT_DEFAULTS[i];
    S.j[i] = {d.min, d.max, d.home, d.zero, d.vmax, d.dir, d.ch, d.pmin, d.pmax};
  }
  strncpy(S.ssid, WIFI_SSID, sizeof(S.ssid) - 1);
  strncpy(S.pass, WIFI_PASS, sizeof(S.pass) - 1);
}

void loadSettings() {
  EEPROM.begin(sizeof(Settings));
  EEPROM.get(0, S);
  if (S.magic != SETTINGS_MAGIC || S.version != SETTINGS_VERSION) {
    loadDefaults();
    Serial.println(F("[cfg] no saved settings, using defaults"));
  } else {
    Serial.println(F("[cfg] loaded settings from flash"));
  }
}

void saveSettings() {
  EEPROM.put(0, S);
  EEPROM.commit();
}

// ============================================================ messaging
void broadcast(const String& s) {
  ws.broadcastTXT(s.c_str(), s.length());
  if (serialActive) Serial.println(s);
}

void sendLog(const String& level, const String& msg) {
  JsonDocument d;
  d["t"] = "log";
  d["l"] = level;
  d["m"] = msg;
  String out;
  serializeJson(d, out);
  broadcast(out);
  if (!serialActive) Serial.println("[" + level + "] " + msg);
}

String fmt1(float v) {
  char b[12];
  dtostrf(v, 0, 1, b);
  return String(b);
}

void sendState() {
  String s = "{\"t\":\"s\",\"p\":[";
  for (int i = 0; i < NUM_JOINTS; i++) s += (i ? "," : "") + fmt1(P.ax[i].pos);
  s += "],\"g\":[";
  for (int i = 0; i < NUM_JOINTS; i++) s += (i ? "," : "") + fmt1(P.ax[i].tgt);
  s += "],\"m\":";
  s += P.moving() ? 1 : 0;
  s += ",\"e\":";
  s += estop ? 1 : 0;
  s += ",\"r\":";
  s += relaxed ? 1 : 0;
  s += "}";
  broadcast(s);
}

void sendCfg() {
  JsonDocument d;
  d["t"] = "cfg";
  d["speed"] = S.speed;
  d["accel"] = S.accel;
  d["sync"] = (bool)S.sync;
  d["bootRelax"] = (bool)S.bootRelax;
  d["osc"] = S.osc;
  JsonArray ja = d["j"].to<JsonArray>();
  for (int i = 0; i < NUM_JOINTS; i++) {
    JsonObject o = ja.add<JsonObject>();
    o["name"] = JOINT_DEFAULTS[i].name;
    o["ch"] = S.j[i].ch;
    o["min"] = S.j[i].min;
    o["max"] = S.j[i].max;
    o["home"] = S.j[i].home;
    o["zero"] = S.j[i].zero;
    o["dir"] = S.j[i].dir;
    o["pmin"] = S.j[i].pmin;
    o["pmax"] = S.j[i].pmax;
    o["vmax"] = S.j[i].vmax;
  }
  String out;
  serializeJson(d, out);
  broadcast(out);
}

String ipString() { return staMode ? WiFi.localIP().toString() : WiFi.softAPIP().toString(); }

void sendInfo() {
  JsonDocument d;
  d["t"] = "info";
  d["fw"] = FW_VERSION;
  d["board"] = BOARD_NAME;
  d["ip"] = ipString();
  d["mode"] = staMode ? "STA" : "AP";
  d["ssid"] = staMode ? String(S.ssid) : String(AP_SSID);
  d["rssi"] = staMode ? WiFi.RSSI() : 0;
  d["pca"] = pcaOk;
  d["heap"] = ESP.getFreeHeap();
  d["up"] = millis() / 1000;
  d["n"] = NUM_JOINTS;
  String out;
  serializeJson(d, out);
  broadcast(out);
}

// ============================================================ servo output
void setupPwm() {
  pwm.begin();
  pwm.setOscillatorFrequency(S.osc);
  pwm.setPWMFreq(SERVO_FREQ);
  // the chip can only hit discrete frequencies — compute the real one
  float prescale = roundf((float)S.osc / (4096.0f * SERVO_FREQ)) - 1;
  float realFreq = (float)S.osc / (4096.0f * (prescale + 1));
  ticksPerUs = 4096.0f * realFreq / 1e6f;
  for (int i = 0; i < NUM_JOINTS; i++) lastTicks[i] = 0xFFFF;
}

uint16_t angleToTicks(int i, float a) {
  const JointCfg& c = S.j[i];
  float us = c.pmin + (a / 180.0f) * (float)(c.pmax - c.pmin);
  return (uint16_t)(us * ticksPerUs + 0.5f);
}

void writeServos(bool force) {
  if (relaxed || !pcaOk) return;
  for (int i = 0; i < NUM_JOINTS; i++) {
    uint16_t t = angleToTicks(i, P.ax[i].pos);
    if (force || t != lastTicks[i]) {
      pwm.setPWM(S.j[i].ch, 0, t);
      lastTicks[i] = t;
    }
  }
}

void relaxAll() {
  relaxed = true;
  P.halt();
  if (!pcaOk) return;
  for (int i = 0; i < NUM_JOINTS; i++) {
    pwm.setPin(S.j[i].ch, 0);  // full off -> servo goes limp
    lastTicks[i] = 0xFFFF;
  }
}

void applyMotionSettings() {
  P.vmax = S.speed;
  P.amax = S.accel;
  P.sync = S.sync;
  for (int i = 0; i < NUM_JOINTS; i++) P.jvmax[i] = S.j[i].vmax;
}

float clampJoint(int i, float a) { return motion::mclamp(a, S.j[i].min, S.j[i].max); }

void clampTargets() {
  for (int i = 0; i < NUM_JOINTS; i++) P.ax[i].tgt = clampJoint(i, P.ax[i].tgt);
}

bool canMove() {
  if (estop) {
    sendLog("warn", "E-STOP active - send reset first");
    return false;
  }
  if (relaxed) {
    sendLog("warn", "Servos relaxed - press Hold first");
    return false;
  }
  return true;
}

// ============================================================ commands
// Shared by WebSocket, HTTP and serial. Mirrored by web/js/sim.js (VirtualArm).
void handleCommand(JsonDocument& d) {
  const char* c = d["c"] | "";

  if (!strcmp(c, "ping")) {
    JsonDocument r;
    r["t"] = "pong";
    r["id"] = d["id"];
    String out;
    serializeJson(r, out);
    broadcast(out);
  } else if (!strcmp(c, "get")) {
    sendInfo();
    sendCfg();
    sendState();
  } else if (!strcmp(c, "set")) {
    if (!canMove()) return;
    JsonArray arr = d["j"].as<JsonArray>();
    mreal t[NUM_JOINTS];
    bool mask[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
      mask[i] = i < (int)arr.size() && !arr[i].isNull();
      t[i] = mask[i] ? clampJoint(i, arr[i].as<float>()) : P.ax[i].tgt;
    }
    P.setTargets(t, mask, d["sync"] | (bool)S.sync);
  } else if (!strcmp(c, "joint") || !strcmp(c, "jog")) {
    if (!canMove()) return;
    int i = d["i"] | -1;
    if (i < 0 || i >= NUM_JOINTS) return;
    float a = !strcmp(c, "jog") ? P.ax[i].tgt + (d["d"] | 0.0f) : (d["a"] | P.ax[i].tgt);
    P.setTarget(i, clampJoint(i, a));
  } else if (!strcmp(c, "home")) {
    if (!canMove()) return;
    mreal t[NUM_JOINTS];
    bool mask[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) {
      t[i] = clampJoint(i, S.j[i].home);
      mask[i] = true;
    }
    P.setTargets(t, mask, true);
    sendLog("info", "Homing");
  } else if (!strcmp(c, "stop")) {
    P.stop();
    clampTargets();
  } else if (!strcmp(c, "estop")) {
    estop = true;
    P.halt();
    sendLog("error", "E-STOP");
  } else if (!strcmp(c, "reset")) {
    estop = false;
    sendLog("info", "E-stop cleared");
  } else if (!strcmp(c, "relax")) {
    relaxAll();
    sendLog("warn", "Servos relaxed (no torque)");
  } else if (!strcmp(c, "hold")) {
    relaxed = false;
    writeServos(true);
    sendLog("info", "Servos holding");
  } else if (!strcmp(c, "speed")) {
    if (!d["v"].isNull()) S.speed = motion::mclamp(d["v"].as<float>(), 1, 720);
    if (!d["a"].isNull()) S.accel = motion::mclamp(d["a"].as<float>(), 1, 5000);
    if (!d["sync"].isNull()) S.sync = d["sync"].as<bool>();
    applyMotionSettings();
    sendCfg();
  } else if (!strcmp(c, "cfg")) {
    if (!d["bootRelax"].isNull()) S.bootRelax = d["bootRelax"].as<bool>();
    bool pwmChanged = false;
    if (!d["osc"].isNull()) {
      S.osc = constrain(d["osc"].as<uint32_t>(), 20000000UL, 30000000UL);
      pwmChanged = true;
    }
    JsonArray arr = d["j"].as<JsonArray>();
    for (JsonObject o : arr) {
      int i = o["i"] | -1;
      if (i < 0 || i >= NUM_JOINTS) continue;
      JointCfg& j = S.j[i];
      if (!o["ch"].isNull()) { j.ch = constrain(o["ch"].as<int>(), 0, 15); pwmChanged = true; }
      if (!o["min"].isNull()) j.min = constrain(o["min"].as<float>(), 0.0f, 180.0f);
      if (!o["max"].isNull()) j.max = constrain(o["max"].as<float>(), 0.0f, 180.0f);
      if (j.max < j.min) { float t = j.min; j.min = j.max; j.max = t; }
      if (!o["home"].isNull()) j.home = o["home"].as<float>();
      j.home = clampJoint(i, j.home);
      if (!o["zero"].isNull()) j.zero = constrain(o["zero"].as<float>(), -180.0f, 360.0f);
      if (!o["dir"].isNull()) j.dir = o["dir"].as<int>() < 0 ? -1 : 1;
      if (!o["pmin"].isNull()) { j.pmin = constrain(o["pmin"].as<int>(), 300, 2900); pwmChanged = true; }
      if (!o["pmax"].isNull()) { j.pmax = constrain(o["pmax"].as<int>(), 300, 2900); pwmChanged = true; }
      if (!o["vmax"].isNull()) j.vmax = constrain(o["vmax"].as<float>(), 1.0f, 720.0f);
    }
    if (pwmChanged && pcaOk) {
      if (!d["osc"].isNull()) setupPwm();
      if (relaxed) relaxAll();
    }
    clampTargets();
    applyMotionSettings();
    for (int i = 0; i < NUM_JOINTS; i++) lastTicks[i] = 0xFFFF;
    writeServos(true);
    sendCfg();
  } else if (!strcmp(c, "save")) {
    saveSettings();
    sendLog("info", "Settings saved to flash");
  } else if (!strcmp(c, "factory")) {
    char ssid[33], pass[65];
    memcpy(ssid, S.ssid, sizeof ssid);  // keep Wi-Fi so we don't lock you out
    memcpy(pass, S.pass, sizeof pass);
    loadDefaults();
    memcpy(S.ssid, ssid, sizeof ssid);
    memcpy(S.pass, pass, sizeof pass);
    saveSettings();
    applyMotionSettings();
    clampTargets();
    sendCfg();
    sendLog("warn", "Factory defaults restored");
  } else if (!strcmp(c, "wifi")) {
    strlcpy(S.ssid, d["ssid"] | "", sizeof(S.ssid));
    strlcpy(S.pass, d["pass"] | "", sizeof(S.pass));
    saveSettings();
    sendLog("warn", String("Wi-Fi set to '") + S.ssid + "', rebooting...");
    delay(300);
    ESP.restart();
  } else if (!strcmp(c, "reboot")) {
    sendLog("warn", "Rebooting...");
    delay(300);
    ESP.restart();
  } else if (!strcmp(c, "scan")) {
    String found;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) found += String(found.length() ? ", 0x" : "0x") + String(a, HEX);
    }
    sendLog("info", "I2C devices: " + (found.length() ? found : String("none")));
  } else {
    sendLog("warn", String("Unknown command: ") + c);
  }
}

void handleJsonText(const char* text, size_t len) {
  JsonDocument d;
  DeserializationError err = deserializeJson(d, text, len);
  if (err) {
    sendLog("warn", String("Bad JSON: ") + err.c_str());
    return;
  }
  handleCommand(d);
}

// Plain-text commands for the Arduino Serial Monitor.
void handleTextCommand(String line) {
  line.trim();
  if (!line.length()) return;
  JsonDocument d;
  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String rest = sp < 0 ? "" : line.substring(sp + 1);
  cmd.toLowerCase();
  if (cmd == "help") {
    Serial.println(F("Commands: status | home | stop | estop | reset | relax | hold | j <0-5> <deg> |"));
    Serial.println(F("          jog <0-5> <+/-deg> | speed <deg/s> | save | factory | scan | reboot |"));
    Serial.println(F("          wifi <ssid> <password>   (or send JSON like {\"c\":\"home\"})"));
    return;
  }
  if (cmd == "status") {
    Serial.printf("fw %s on %s  ip %s (%s)  pca9685 %s  estop %d relaxed %d\n", FW_VERSION, BOARD_NAME,
                  ipString().c_str(), staMode ? "STA" : "AP", pcaOk ? "OK" : "MISSING", estop, relaxed);
    for (int i = 0; i < NUM_JOINTS; i++)
      Serial.printf("  %d %-12s ch%-2d pos %6.1f tgt %6.1f  [%g..%g] home %g\n", i, JOINT_DEFAULTS[i].name,
                    S.j[i].ch, P.ax[i].pos, P.ax[i].tgt, S.j[i].min, S.j[i].max, S.j[i].home);
    return;
  }
  if (cmd == "j" || cmd == "jog") {
    int sp2 = rest.indexOf(' ');
    d["c"] = cmd == "j" ? "joint" : "jog";
    d["i"] = rest.substring(0, sp2).toInt();
    d[cmd == "j" ? "a" : "d"] = rest.substring(sp2 + 1).toFloat();
  } else if (cmd == "speed") {
    d["c"] = "speed";
    d["v"] = rest.toFloat();
  } else if (cmd == "wifi") {
    int sp2 = rest.indexOf(' ');
    d["c"] = "wifi";
    d["ssid"] = sp2 < 0 ? rest : rest.substring(0, sp2);
    d["pass"] = sp2 < 0 ? "" : rest.substring(sp2 + 1);
  } else {
    d["c"] = cmd;
  }
  handleCommand(d);
}

void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (serialLine.length()) {
        if (serialLine[0] == '{') {
          serialActive = true;
          handleJsonText(serialLine.c_str(), serialLine.length());
        } else {
          handleTextCommand(serialLine);
        }
      }
      serialLine = "";
    } else if (serialLine.length() < 1024) {
      serialLine += ch;
    }
  }
}

// ============================================================ network
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      sendInfo();
      sendCfg();
      sendState();
      break;
    case WStype_TEXT:
      handleJsonText((const char*)payload, len);
      break;
    case WStype_DISCONNECTED:
      // last controller gone -> don't keep moving blindly
      if (ws.connectedClients() == 0 && P.moving()) {
        P.stop();
        clampTargets();
      }
      break;
    default:
      break;
  }
}

bool serveAsset(String path) {
  if (path == "/") path = "/index.html";
  for (size_t i = 0; i < WEB_ASSET_COUNT; i++) {
    if (path == WEB_ASSETS[i].path) {
      server.sendHeader("Content-Encoding", "gzip");
      server.sendHeader("Cache-Control", "max-age=600");
      server.send_P(200, WEB_ASSETS[i].mime, (PGM_P)WEB_ASSETS[i].data, WEB_ASSETS[i].len);
      return true;
    }
  }
  return false;
}

void setupHttp() {
  server.on("/api/cmd", HTTP_POST, []() {
    String body = server.arg("plain");
    handleJsonText(body.c_str(), body.length());
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.on("/api/state", HTTP_GET, []() {
    String s = "{\"p\":[";
    for (int i = 0; i < NUM_JOINTS; i++) s += (i ? "," : "") + fmt1(P.ax[i].pos);
    s += "],\"m\":" + String(P.moving() ? 1 : 0) + ",\"e\":" + String(estop ? 1 : 0) + "}";
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", s);
  });
  server.onNotFound([]() {
    if (!serveAsset(server.uri())) server.send(404, "text/plain", "Not found");
  });
  server.begin();
}

void setupWifi() {
#if defined(ESP8266)
  WiFi.persistent(false);
  WiFi.hostname(HOSTNAME);
#else
  WiFi.setHostname(HOSTNAME);
#endif
  if (strlen(S.ssid)) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(S.ssid, S.pass);
    Serial.printf("[wifi] connecting to '%s'", S.ssid);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    staMode = WiFi.status() == WL_CONNECTED;
  }
  if (!staMode) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[wifi] access point '%s' (password '%s')\n", AP_SSID, AP_PASS);
  }
  Serial.printf("[wifi] open http://%s/  or  http://%s.local/\n", ipString().c_str(), HOSTNAME);
  if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", HTTP_PORT);
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.onStart([]() {
    estop = true;
    P.halt();
  });
  ArduinoOTA.begin();
}

// ============================================================ main
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(50);
  Serial.printf("\n\nRoboArm %s on %s\n", FW_VERSION, BOARD_NAME);

  loadSettings();
  applyMotionSettings();

  mreal start[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; i++) start[i] = clampJoint(i, S.j[i].home);
  P.init(start);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(PCA9685_ADDR);
  pcaOk = Wire.endTransmission() == 0;
  if (pcaOk) {
    setupPwm();
    if (S.bootRelax) relaxAll();
    else writeServos(true);
    Serial.println(F("[pca] PCA9685 found"));
  } else {
    Serial.printf("[pca] PCA9685 NOT found at 0x%02X - check SDA=%d SCL=%d, VCC, GND\n", PCA9685_ADDR, I2C_SDA, I2C_SCL);
  }

  setupWifi();
  setupHttp();
  ws.begin();
  ws.onEvent(onWsEvent);
  Serial.println(F("Ready. Type 'help' for serial commands."));
}

void loop() {
  static uint32_t lastCtl = micros();
  static uint32_t lastState = 0;
  static bool wasMoving = true;
  const uint32_t period = 1000000UL / CONTROL_HZ;

  server.handleClient();
  ws.loop();
  pollSerial();
  ArduinoOTA.handle();
#if defined(ESP8266)
  MDNS.update();
#endif

  uint32_t now = micros();
  int steps = 0;
  while ((uint32_t)(now - lastCtl) >= period && steps < 5) {
    P.update(1.0f / CONTROL_HZ);
    lastCtl += period;
    steps++;
  }
  if ((uint32_t)(now - lastCtl) >= period) lastCtl = now;  // fell far behind, resync
  if (steps) writeServos(false);

  bool moving = P.moving();
  uint32_t ms = millis();
  uint32_t interval = (moving || wasMoving) ? 1000 / STATE_HZ : 1000;
  if (ms - lastState >= interval) {
    lastState = ms;
    sendState();
    wasMoving = moving;
  }
}
