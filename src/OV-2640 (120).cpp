#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>

#include "esp_task_wdt.h"
#include "esp_idf_version.h"

// [translate:Wi‑Fi точки доступа Raspberry Pi]
static const char* WIFI_SSID = "Raspberry";
static const char* WIFI_PASS = "12345678";

// [translate:Flask сервер на Raspberry Pi]
static const char* SERVER_HOST = "192.168.4.1";
static const uint16_t SERVER_PORT = 8000;
static const char* SERVER_PATH = "/upload";

// [translate:ID камеры для раздельных папок на сервере]
static const char* CAM_ID = "cam3";

// [translate:Интервалы]
static const uint32_t CAPTURE_PERIOD_SEC = 600;   // 10 минут
static const uint32_t FAIL_SLEEP_SEC     = 600;   // гибрид: при провале ждём до следующего слота
static const uint8_t  UPLOAD_RETRIES     = 3;

// [translate:Пины (AI-Thinker ESP32-CAM + OV2640)]
// Важно: для OV2640 на AI-Thinker обычно используется тот же pinout камеры,
// но если у вас нестандартная плата — пины могут отличаться.
#define LED_FLASH_GPIO    GPIO_NUM_4
#define STATUS_LED_GPIO   GPIO_NUM_33

static void blinkStatus(uint8_t times, uint16_t on_ms, uint16_t off_ms) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_GPIO, LOW);
    delay(on_ms);
    digitalWrite(STATUS_LED_GPIO, HIGH);
    delay(off_ms);
  }
}

static void goToSleep(uint32_t seconds) {
  Serial.printf("Going to deep sleep for %u seconds...\n", seconds);
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

static void initTaskWdt() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t twdt_config = {};
  twdt_config.timeout_ms = 8000;
  twdt_config.idle_core_mask = 0;
  twdt_config.trigger_panic = true;
  esp_task_wdt_init(&twdt_config);
#else
  esp_task_wdt_init(8, true);
#endif
  esp_task_wdt_add(NULL);
}

static bool connectWiFi(uint32_t timeout_ms) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
    delay(200);
    esp_task_wdt_reset();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("WiFi connect FAILED");
  return false;
}

static bool postMultipartJpeg(camera_fb_t* fb) {
  if (!fb || !fb->buf || fb->len == 0) return false;

  WiFiClient client;
  client.setTimeout(15000);

  if (!client.connect(SERVER_HOST, SERVER_PORT)) {
    Serial.println("TCP connect FAILED");
    return false;
  }

  const String boundary = "----esp32camBoundary7MA4YWxk";
  const String part_cam =
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"cam_id\"\r\n\r\n" +
    String(CAM_ID) + "\r\n";

  const String part_file_head =
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"file\"; filename=\"frame.jpg\"\r\n"
    "Content-Type: image/jpeg\r\n\r\n";

  const String tail = "\r\n--" + boundary + "--\r\n";

  const uint32_t contentLength =
      (uint32_t)part_cam.length() +
      (uint32_t)part_file_head.length() +
      (uint32_t)fb->len +
      (uint32_t)tail.length();

  client.print(String("POST ") + SERVER_PATH + " HTTP/1.1\r\n");
  client.print(String("Host: ") + SERVER_HOST + ":" + String(SERVER_PORT) + "\r\n");
  client.print("Connection: close\r\n");
  client.print("Content-Type: multipart/form-data; boundary=" + boundary + "\r\n");
  client.print("Content-Length: " + String(contentLength) + "\r\n\r\n");

  client.print(part_cam);
  client.print(part_file_head);

  const uint8_t* p = fb->buf;
  size_t remaining = fb->len;
  while (remaining > 0) {
    esp_task_wdt_reset();
    size_t chunk = remaining > 1024 ? 1024 : remaining;
    size_t written = client.write(p, chunk);
    if (written == 0) {
      Serial.println("Socket write FAILED");
      client.stop();
      return false;
    }
    p += written;
    remaining -= written;
  }

  client.print(tail);

  String statusLine = client.readStringUntil('\n');
  statusLine.trim();
  Serial.print("HTTP status line: ");
  Serial.println(statusLine);

  bool ok = statusLine.startsWith("HTTP/1.1 200") || statusLine.startsWith("HTTP/1.0 200");
  client.stop();
  return ok;
}

// [translate:Настройки качества для OV2640: максимум детализации без роста артефактов]
static void applyOv2640QualityProfile() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;

  // Нейтральная база
  s->set_brightness(s, 0);       // -2..2 [web:4]
  s->set_contrast(s, 0);         // -2..2 [web:4]
  s->set_saturation(s, 0);       // -2..2 [web:4]
  s->set_special_effect(s, 0);   // [web:4]

  // Автобаланс/автоэкспозиция включены (для дневной облачности стабильнее)
  s->set_whitebal(s, 1);         // [web:4]
  s->set_awb_gain(s, 1);         // [web:4]
  s->set_exposure_ctrl(s, 1);    // [web:4]
  s->set_gain_ctrl(s, 1);        // [web:4]

  // Коррекции и подавление дефектов
  s->set_lenc(s, 1);             // [web:4]
  s->set_raw_gma(s, 1);          // [web:4]
  s->set_wpc(s, 1);              // white pixel correction [web:4]
  s->set_bpc(s, 1);              // bad pixel correction [web:4]
  s->set_dcw(s, 1);              // downsize/crop pipeline [web:4]

  // AEC2 иногда даёт “пампинг” экспозиции на небе; оставляем OFF для предсказуемости
  s->set_aec2(s, 0);             // [web:4]

  // Ограничение усиления — уменьшает шум/полосы в сумерках
  s->set_gainceiling(s, (gainceiling_t)2); // [web:4]

  s->set_colorbar(s, 0);         // [web:4]
}

static void startCameraOv2640() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  // AI-Thinker camera pins
  config.pin_pwdn     = 32;
  config.pin_reset    = -1;
  config.pin_xclk     = 0;
  config.pin_sccb_sda = 26;
  config.pin_sccb_scl = 27;
  config.pin_d7       = 35;
  config.pin_d6       = 34;
  config.pin_d5       = 39;
  config.pin_d4       = 36;
  config.pin_d3       = 21;
  config.pin_d2       = 19;
  config.pin_d1       = 18;
  config.pin_d0       = 5;
  config.pin_vsync    = 25;
  config.pin_href     = 23;
  config.pin_pclk     = 22;

  // Для OV2640 обычно устойчиво 20MHz; если увидите битые JPEG/сбои — снизим до 10–16MHz. [web:36]
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (!psramFound()) {
    Serial.println("PSRAM NOT FOUND. HALTING.");
    while (true) delay(1000);
  }

  // OV2640 максимум — UXGA (1600x1200) [web:35][web:4]
  config.frame_size   = FRAMESIZE_UXGA;

  // Ключевой баланс:
  // Слишком низкие значения могут давать “битые/рваные” JPEG на высоких деталях/движении,
  // поэтому берём 10 как максимально практичное, а если будут артефакты — поднимем до 11–12. [web:4][web:36]
  config.jpeg_quality = 10;

  // При PSRAM обычно используют 2 буфера, но если появятся битые кадры — переключим на 1. [web:4][web:36]
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true) delay(1000);
  }

  Serial.println("OV2640 camera initialized.");
  applyOv2640QualityProfile();
  Serial.println("OV2640 quality profile applied.");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LED_FLASH_GPIO, OUTPUT);
  digitalWrite(LED_FLASH_GPIO, LOW);

  pinMode(STATUS_LED_GPIO, OUTPUT);
  digitalWrite(STATUS_LED_GPIO, HIGH);

  initTaskWdt();
  blinkStatus(2, 120, 120);

  startCameraOv2640();

  digitalWrite(STATUS_LED_GPIO, LOW);
  bool wifiOk = connectWiFi(15000);
  if (!wifiOk) {
    digitalWrite(STATUS_LED_GPIO, HIGH);
    goToSleep(FAIL_SLEEP_SEC);
  }

  bool uploaded = false;
  for (uint8_t attempt = 1; attempt <= UPLOAD_RETRIES; attempt++) {
    esp_task_wdt_reset();
    Serial.printf("Capture+upload attempt %u/%u...\n", attempt, UPLOAD_RETRIES);

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Capture FAILED");
      delay(400 * attempt);
      continue;
    }

    Serial.printf("Captured %u bytes\n", fb->len);

    bool ok = postMultipartJpeg(fb);
    esp_camera_fb_return(fb);

    if (ok) {
      uploaded = true;
      Serial.println("Upload OK (HTTP 200).");
      break;
    }

    Serial.println("Upload FAILED, backoff...");
    delay(500 * attempt);
  }

  digitalWrite(STATUS_LED_GPIO, HIGH);

  if (uploaded) goToSleep(CAPTURE_PERIOD_SEC);
  goToSleep(FAIL_SLEEP_SEC);
}

void loop() {
  // [translate:Не используется]
}
