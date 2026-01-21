#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <time.h>

// ------------------- CONFIG -------------------
static const char* WIFI_SSID = "Raspberry";
static const char* WIFI_PASS = "12345678";

static const char* SERVER_HOST = "192.168.4.1";
static const uint16_t SERVER_PORT = 8000;

static const char* CAM_ID = "cam160";   
static const uint32_t CAPTURE_PERIOD_SEC = 600;   
static const uint32_t FAIL_SLEEP_SEC = 120;
static const uint8_t UPLOAD_RETRIES = 3;

static const uint32_t WIFI_TIMEOUT_MS = 15000;
static const uint32_t CMD_TOTAL_WAIT_MS = 45000;
static const uint32_t ACK_TOTAL_WAIT_MS = 45000;
static const uint32_t SNTPTIMEOUT_MS = 8000;

static const bool WIFI_DISABLE_SLEEP = true;
static const bool WIFI_MAX_TXPOWER = true;

// ------------------- GPIO -------------------
#define LED_FLASH_GPIO GPIO_NUM_4
#define STATUS_LED_GPIO GPIO_NUM_33   

// ------------------- helpers -------------------
static uint32_t ms() { return millis(); }

static void goToSleep(uint32_t seconds) {
  Serial.printf("[SLEEP] %u sec\n", (unsigned)seconds);
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

static void blinkStatus(uint8_t times, uint16_t onms, uint16_t offms) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_GPIO, LOW);
    delay(onms);
    digitalWrite(STATUS_LED_GPIO, HIGH);
    delay(offms);
  }
}

static bool readLine(WiFiClient &c, String &line, uint32_t timeoutMs) {
  line = "";
  uint32_t t0 = ms();
  while (ms() - t0 < timeoutMs) {
    while (c.available()) {
      char ch = (char)c.read();
      if (ch == '\r') continue;
      if (ch == '\n') return true;
      line += ch;
    }
    if (!c.connected()) return line.length() > 0;
    delay(1);
  }
  return false;
}

struct HttpResp {
  int status = -1;
  String body;
};

static bool httpRequestRaw(const String &req, HttpResp &out, uint32_t totalTimeoutMs) {
  out = HttpResp();
  WiFiClient client;
  client.setTimeout(2000);
  if (!client.connect(SERVER_HOST, SERVER_PORT)) return false;

  client.print(req);

  uint32_t t0 = ms();
  String statusLine;
  if (!readLine(client, statusLine, 2000)) { client.stop(); return false; }

  int sp = statusLine.indexOf(' ');
  if (sp < 0) { client.stop(); return false; }
  int sp2 = statusLine.indexOf(' ', sp + 1);
  String codeStr = (sp2 > sp) ? statusLine.substring(sp + 1, sp2) : statusLine.substring(sp + 1);
  out.status = codeStr.toInt();

  int contentLen = -1;
  while (true) {
    String h;
    if (!readLine(client, h, 2000)) { client.stop(); return false; }
    if (h.length() == 0) break;
    String hl = h; hl.toLowerCase();
    if (hl.startsWith("content-length:")) contentLen = hl.substring(15).toInt();
    if (ms() - t0 > totalTimeoutMs) { client.stop(); return false; }
  }

  String body;
  if (contentLen >= 0) body.reserve(contentLen + 8);

  if (contentLen >= 0) {
    while ((int)body.length() < contentLen && (ms() - t0 < totalTimeoutMs)) {
      while (client.available() && (int)body.length() < contentLen) body += (char)client.read();
      if (!client.connected() && !client.available()) break;
      delay(1);
    }
  } else {
    while (ms() - t0 < totalTimeoutMs) {
      while (client.available()) body += (char)client.read();
      if (!client.connected() && !client.available()) break;
      delay(1);
    }
  }

  client.stop();
  body.trim();
  out.body = body;
  return true;
}

static bool jsonFindInt(const String &json, const char *key, int &out) {
  String pat = String("\"") + key + "\":";
  int idx = json.indexOf(pat);
  if (idx < 0) return false;
  int colon = json.indexOf(':', idx);
  if (colon < 0) return false;
  colon++;
  while (colon < (int)json.length() && (json[colon] == ' ' || json[colon] == '\"')) colon++;
  String num = "";
  while (colon < (int)json.length() && isdigit((unsigned char)json[colon])) num += json[colon++];
  if (num.length() == 0) return false;
  out = num.toInt();
  return true;
}

static bool jsonFindBool(const String &json, const char *key, bool &out) {
  String pat = String("\"") + key + "\":";
  int idx = json.indexOf(pat);
  if (idx < 0) return false;
  int colon = json.indexOf(':', idx);
  if (colon < 0) return false;
  colon++;
  while (colon < (int)json.length() && (json[colon] == ' ')) colon++;
  if (json.substring(colon).startsWith("true")) { out = true; return true; }
  if (json.substring(colon).startsWith("false")) { out = false; return true; }
  return false;
}

static void wifiTuneAfterConnect() {
  if (WIFI_DISABLE_SLEEP) WiFi.setSleep(false);
  if (WIFI_MAX_TXPOWER) WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

static bool connectWiFi(uint32_t timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = ms();
  while (WiFi.status() != WL_CONNECTED && (ms() - t0 < timeoutMs)) delay(50);
  if (WiFi.status() != WL_CONNECTED) return false;
  wifiTuneAfterConnect();
  return true;
}

static bool syncTimeSNTP() {
  configTime(0, 0, SERVER_HOST);
  uint32_t t0 = ms();
  struct tm ti;
  while (ms() - t0 < SNTPTIMEOUT_MS) {
    if (getLocalTime(&ti, 200)) return true;
    delay(50);
  }
  return false;
}

// ------------------- Camera init OV2640 (AI Thinker) -------------------
static void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sccb_sda = 26;  // или pin_sccb_sda в новых API
  config.pin_sccb_scl = 27;  // или pin_sccb_scl
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (!psramFound()) {
    Serial.println("PSRAM NOT FOUND. HALTING.");
    while (true) delay(1000);
  }

  config.frame_size = FRAMESIZE_VGA;     // Или SVGA для теста
  config.jpeg_quality = 10;              // Улучшенное качество
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }
  Serial.println("Camera initialized.");

  // ← ВСТАВЬТЕ ЗДЕСЬ БЛОК НАСТРОЕК СЕНСОРА:
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, 1);      // Автобаланс
    s->set_awb_gain(s, 1);      // Gain по каналам
    s->set_gain_ctrl(s, 1);     // Автогейн
    s->set_exposure_ctrl(s, 1); // Автоэкспозиция
    s->set_aec2(s, 0);          // AEC алгоритм 1
    s->set_lenc(s, 1);          // Коррекция линзы
    s->set_raw_gma(s, 1);       // Гамма
    s->set_wpc(s, 1);           // White pixel correction
    s->set_bpc(s, 1);           // Black pixel correction
    s->set_dcw(s, 1);           // Downsize enabler
    s->set_gainceiling(s, (gainceiling_t)2); // Gain ceiling 2x (умеренно)
  }
  Serial.println("Sensor HQ settings applied.");
}

 // ------------------- API calls -------------------
static bool postHello(int &cycleIdOut, int &serverMsOut) {
  String payload = String("{\"deviceid\":\"") + CAM_ID + "\"}";
  String req =
      String("POST /hello HTTP/1.1\r\n") +
      "Host: " + SERVER_HOST + ":" + String(SERVER_PORT) + "\r\n" +
      "Connection: close\r\n" +
      "Content-Type: application/json\r\n" +
      "Content-Length: " + String(payload.length()) + "\r\n\r\n" +
      payload;

  HttpResp r;
  if (!httpRequestRaw(req, r, 5000)) return false;
  if (r.status != 200) return false;
  int cid = -1, sm = -1;
  if (!jsonFindInt(r.body, "cycle_id", cid)) return false;
  if (!jsonFindInt(r.body, "server_ms", sm)) return false;
  cycleIdOut = cid;
  serverMsOut = sm;
  return true;
}

static bool waitCmd(int cycleId, int &tCaptureMsOut, int &serverMsOut) {
  uint32_t t0 = ms();
  while (ms() - t0 < CMD_TOTAL_WAIT_MS) {
    String path = String("/waitcmd?deviceid=") + CAM_ID + "&cycle_id=" + String(cycleId);
    String req =
        String("GET ") + path + " HTTP/1.1\r\n" +
        "Host: " + String(SERVER_HOST) + ":" + String(SERVER_PORT) + "\r\n" +
        "Connection: close\r\n\r\n";
    HttpResp r;
    if (!httpRequestRaw(req, r, 32000)) { delay(50); continue; }
    if (r.status != 200) { delay(50); continue; }

    if (r.body.indexOf("\"type\":\"CAPTURE_AT\"") >= 0) {
      int tcap = -1, sm = -1;
      if (!jsonFindInt(r.body, "t_capture_ms", tcap)) return false;
      if (!jsonFindInt(r.body, "server_ms", sm)) return false;
      tCaptureMsOut = tcap;
      serverMsOut = sm;
      return true;
    }
    delay(100);
  }
  return false;
}

static bool waitAck(int cycleId, bool &sleepOut) {
  uint32_t t0 = ms();
  while (ms() - t0 < ACK_TOTAL_WAIT_MS) {
    String path = String("/waitack?deviceid=") + CAM_ID + "&cycle_id=" + String(cycleId);
    String req =
        String("GET ") + path + " HTTP/1.1\r\n" +
        "Host: " + String(SERVER_HOST) + ":" + String(SERVER_PORT) + "\r\n" +
        "Connection: close\r\n\r\n";
    HttpResp r;
    if (!httpRequestRaw(req, r, 32000)) { delay(50); continue; }
    if (r.status != 200) { delay(50); continue; }
    bool s = false;
    if (!jsonFindBool(r.body, "sleep", s)) { delay(50); continue; }
    sleepOut = s;
    return true;
  }
  return false;
}

static bool postMultipartJpeg(camera_fb_t *fb, int cycleId,
                             const String &metaJson) {
  if (!fb || !fb->buf || fb->len == 0) return false;

  WiFiClient client;
  client.setTimeout(2000);
  if (!client.connect(SERVER_HOST, SERVER_PORT)) return false;

  const String boundary = "----esp32Boundary7MA4YWxk";
  const String path = String("/upload?camid=") + CAM_ID + "&cycle_id=" + String(cycleId);

  const String partMeta =
      String("--") + boundary + "\r\n" +
      "Content-Disposition: form-data; name=\"meta\"\r\n" +
      "Content-Type: application/json\r\n\r\n" +
      metaJson + "\r\n";

  const String partFileHead =
      String("--") + boundary + "\r\n" +
      "Content-Disposition: form-data; name=\"file\"; filename=\"frame.jpg\"\r\n" +
      "Content-Type: image/jpeg\r\n\r\n";

  const String tail = String("\r\n--") + boundary + "--\r\n";

  uint32_t contentLength = partMeta.length() + partFileHead.length() + fb->len + tail.length();

  client.print(String("POST ") + path + " HTTP/1.1\r\n");
  client.print(String("Host: ") + SERVER_HOST + ":" + String(SERVER_PORT) + "\r\n");
  client.print("Connection: close\r\n");
  client.print(String("Content-Type: multipart/form-data; boundary=") + boundary + "\r\n");
  client.print(String("Content-Length: ") + String(contentLength) + "\r\n\r\n");

  client.print(partMeta);
  client.print(partFileHead);

  const uint8_t *p = fb->buf;
  size_t remaining = fb->len;
  while (remaining > 0) {
    size_t chunk = remaining > 1024 ? 1024 : remaining;
    size_t written = client.write(p, chunk);
    if (written == 0) { client.stop(); return false; }
    p += written;
    remaining -= written;
    delay(1);
  }

  client.print(tail);

  String statusLine;
  if (!readLine(client, statusLine, 2000)) { client.stop(); return false; }
  statusLine.trim();
  client.stop();
  return (statusLine.startsWith("HTTP/1.1 200") || statusLine.startsWith("HTTP/1.0 200"));
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(LED_FLASH_GPIO, OUTPUT);
  digitalWrite(LED_FLASH_GPIO, LOW);
  pinMode(STATUS_LED_GPIO, OUTPUT);
  digitalWrite(STATUS_LED_GPIO, HIGH);

  blinkStatus(2, 120, 120);
  startCamera();

  uint32_t tBoot = ms();

  uint32_t t0 = ms();
  if (!connectWiFi(WIFI_TIMEOUT_MS)) goToSleep(FAIL_SLEEP_SEC);
  Serial.printf("[TIMING] wifi_connect_ms=%u rssi=%d\n", (unsigned)(ms() - t0), WiFi.RSSI());

  t0 = ms();
  bool timeOk = syncTimeSNTP();
  Serial.printf("[TIMING] sntp_ms=%u ok=%s\n", (unsigned)(ms() - t0), timeOk ? "true" : "false");

  int cycleId = -1, helloServerMs = -1;
  t0 = ms();
  bool helloOk = postHello(cycleId, helloServerMs);
  Serial.printf("[TIMING] hello_ms=%u ok=%s cycle_id=%d\n",
                (unsigned)(ms() - t0), helloOk ? "true" : "false", cycleId);
  if (!helloOk) goToSleep(FAIL_SLEEP_SEC);

  int tCaptureMs = -1, cmdServerMs = -1;
  t0 = ms();
  bool cmdOk = waitCmd(cycleId, tCaptureMs, cmdServerMs);
  Serial.printf("[TIMING] waitcmd_ms=%u ok=%s t_capture_ms=%d\n",
                (unsigned)(ms() - t0), cmdOk ? "true" : "false", tCaptureMs);
  if (!cmdOk) goToSleep(FAIL_SLEEP_SEC);

  // Выравнивание CAPTURE_AT: оценим смещение local_ms - server_ms
  int offsetMs = (int)ms() - cmdServerMs;
  int waitMs = tCaptureMs + offsetMs - (int)ms();
  if (waitMs > 0) delay((uint32_t)waitMs);

  // CAPTURE
  t0 = ms();
  camera_fb_t *fb = esp_camera_fb_get();
  uint32_t captureMs = ms() - t0;
  if (!fb) {
    Serial.println("[ERR] Capture FAILED");
    goToSleep(FAIL_SLEEP_SEC);
  }
  Serial.printf("[TIMING] capture_ms=%u jpeg_bytes=%u\n", (unsigned)captureMs, (unsigned)fb->len);

  // META JSON
  String meta = "{";
  meta += "\"cam_id\":\"" + String(CAM_ID) + "\",";
  meta += "\"cycle_id\":" + String(cycleId) + ",";
  meta += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  meta += "\"local_ms\":" + String((int)ms()) + ",";
  meta += "\"cmd_server_ms\":" + String(cmdServerMs) + ",";
  meta += "\"t_capture_ms\":" + String(tCaptureMs) + ",";
  meta += "\"capture_ms\":" + String((unsigned)captureMs) + ",";
  meta += "\"jpeg_bytes\":" + String((unsigned)fb->len) + ",";
  meta += "\"time_valid\":" + String(timeOk ? "true" : "false");
  meta += "}";

  // UPLOAD
  bool uploaded = false;
  uint32_t uploadStart = ms();
  for (uint8_t attempt = 1; attempt <= UPLOAD_RETRIES; attempt++) {
    uint32_t ta = ms();
    bool ok = postMultipartJpeg(fb, cycleId, meta);
    uint32_t dt = ms() - ta;
    Serial.printf("[TIMING] upload_try=%u dt_ms=%u ok=%s\n",
                  attempt, (unsigned)dt, ok ? "true" : "false");
    if (ok) { uploaded = true; break; }
    delay(200 * attempt);
  }
  esp_camera_fb_return(fb);
  Serial.printf("[TIMING] upload_total_ms=%u uploaded=%s\n",
                (unsigned)(ms() - uploadStart), uploaded ? "true" : "false");
  if (!uploaded) goToSleep(FAIL_SLEEP_SEC);

  // ACK
  bool sleepFlag = false;
  t0 = ms();
  bool ackOk = waitAck(cycleId, sleepFlag);
  Serial.printf("[TIMING] waitack_ms=%u ok=%s sleep=%s\n",
                (unsigned)(ms() - t0), ackOk ? "true" : "false", sleepFlag ? "true" : "false");

  Serial.printf("[TIMING] total_awake_ms=%u\n", (unsigned)(ms() - tBoot));
  goToSleep(CAPTURE_PERIOD_SEC);
}

void loop() {}
