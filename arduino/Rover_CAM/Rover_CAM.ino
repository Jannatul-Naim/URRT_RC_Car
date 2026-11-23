/*
 ===============================================================================
   ECO ROVER — ESP32-CAM (AI-Thinker) CAMERA BOARD (Arduino IDE sketch)

   What this board does:
     - Streams live video over Wi-Fi. Nothing else.
     - The laptop (laptop/server.py) pulls this stream, decodes QR codes,
       detects motion and (later) classifies hazmat signs, then shows the
       video inside the dashboard.
     - The pan servo is NOT on this board any more: it is on the S3, GPIO 8.

   Endpoints:
     http://<CAM-IP>/            short help text
     http://<CAM-IP>/capture     one JPEG picture
     http://<CAM-IP>/status      JSON: sensor, frame size, uptime, Wi-Fi signal
     http://<CAM-IP>/flash?on=1  flash LED on (on=0 for off), helps QR in the dark
     http://<CAM-IP>:81/stream   MJPEG live stream (this is what the laptop reads)

   Upload (Arduino IDE):
     Board: "AI Thinker ESP32-CAM"   Partition scheme: "Huge APP (3MB No OTA)"
     Wire a USB-serial adapter to U0R/U0T, hold IO0 to GND while resetting
     to enter flash mode, then remove the IO0 jumper and reset to run.

   Works with Arduino-ESP32 core 2.x and 3.x.
 ===============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"

// =============================================================================
// 1. CONFIGURATION
// =============================================================================
// Wi-Fi name and password live in secrets.h (git-ignored). Copy
// secrets.example.h to secrets.h in this folder and fill it in.
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #error "Copy secrets.example.h to secrets.h in this sketch folder and set your Wi-Fi name and password"
#endif
#define MDNS_HOSTNAME    "rover-cam"     // http://rover-cam.local

// Optional fixed IP so the laptop always finds the camera at the same address
#define USE_STATIC_IP    false
#define STATIC_IP        192, 168, 1, 51
#define STATIC_GATEWAY   192, 168, 1, 1
#define STATIC_SUBNET    255, 255, 255, 0

#define JPEG_QUALITY     12              // OV2640: 10 (best) .. 63 (smallest)
#define SOFT_JPEG_QUALITY 70             // other sensors, software JPEG: 1..100
// The team's camera is a GC2145 (no hardware JPEG), so frames are JPEG-encoded
// in software. QVGA = 320x240, about 8-10 fps. For sharper QR codes try
// FRAMESIZE_HVGA (480x320) or FRAMESIZE_VGA (640x480): slower, but more detail.
#define SOFT_FRAME_SIZE  FRAMESIZE_QVGA
#define STREAM_MAX_FPS   20

// AI-Thinker ESP32-CAM camera pins
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22
#define FLASH_LED_PIN     4

// =============================================================================
// 2. CAMERA
// =============================================================================
bool hardwareJpeg = true;   // false when the sensor cannot make JPEG (e.g. GC2145)

bool initCamera(pixformat_t format, framesize_t size) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = format;
  c.frame_size = size;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;     // always send the newest frame (low lag)
  return esp_camera_init(&c) == ESP_OK;
}

void setupCamera() {
  // OV2640 makes JPEG in hardware: VGA 640x480, good for reading QR codes.
  if (initCamera(PIXFORMAT_JPEG, FRAMESIZE_VGA)) {
    hardwareJpeg = true;
    Serial.println("[CAM] Hardware JPEG, 640x480");
  } else {
    // Sensors without JPEG (GC2145 family): RGB565, JPEG made in software.
    esp_camera_deinit();
    if (initCamera(PIXFORMAT_RGB565, SOFT_FRAME_SIZE)) {
      hardwareJpeg = false;
      Serial.println("[CAM] No hardware JPEG (GC2145?): RGB565 + software JPEG");
    } else {
      Serial.println("[CAM] Camera init FAILED. Check the ribbon cable, then reset.");
      delay(3000);
      ESP.restart();
    }
  }
  sensor_t* s = esp_camera_sensor_get();
  Serial.printf("[CAM] Sensor PID 0x%02X\n", s->id.PID);
}

// Gets one frame as JPEG. Call releaseJpeg() with the same arguments afterwards.
bool grabJpeg(camera_fb_t*& fb, uint8_t*& buf, size_t& len) {
  fb = esp_camera_fb_get();
  if (!fb) return false;
  if (hardwareJpeg) {
    buf = fb->buf;
    len = fb->len;
    return true;
  }
  bool ok = frame2jpg(fb, SOFT_JPEG_QUALITY, &buf, &len);
  if (!ok) { esp_camera_fb_return(fb); fb = nullptr; }
  return ok;
}

void releaseJpeg(camera_fb_t* fb, uint8_t* buf) {
  if (!hardwareJpeg && buf) free(buf);
  if (fb) esp_camera_fb_return(fb);
}

// =============================================================================
// 3. WEB SERVERS (port 80: control, port 81: stream)
// =============================================================================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t controlServer = nullptr;
httpd_handle_t streamServer = nullptr;
volatile float streamFps = 0;

esp_err_t rootHandler(httpd_req_t* req) {
  const char* msg = "Eco Rover ESP32-CAM\n/capture  /status  /flash?on=1\nLive stream: port 81 /stream\n";
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, msg, HTTPD_RESP_USE_STRLEN);
}

esp_err_t captureHandler(httpd_req_t* req) {
  camera_fb_t* fb; uint8_t* buf; size_t len;
  if (!grabJpeg(fb, buf, len)) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t res = httpd_resp_send(req, (const char*)buf, len);
  releaseJpeg(fb, buf);
  return res;
}

esp_err_t statusHandler(httpd_req_t* req) {
  sensor_t* s = esp_camera_sensor_get();
  char json[200];
  snprintf(json, sizeof(json),
           "{\"sensor_pid\":%u,\"hw_jpeg\":%s,\"stream_fps\":%.1f,\"uptime_ms\":%lu,\"rssi\":%d,\"heap\":%u}",
           s ? s->id.PID : 0, hardwareJpeg ? "true" : "false", streamFps,
           millis(), WiFi.RSSI(), ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

esp_err_t flashHandler(httpd_req_t* req) {
  char query[16] = "", val[4] = "0";
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    httpd_query_key_value(query, "on", val, sizeof(val));
  }
  digitalWrite(FLASH_LED_PIN, val[0] == '1' ? HIGH : LOW);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

// Sends JPEG frames forever until the client (the laptop) disconnects.
esp_err_t streamHandler(httpd_req_t* req) {
  if (httpd_resp_set_type(req, STREAM_CONTENT_TYPE) != ESP_OK) return ESP_FAIL;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  Serial.println("[STREAM] Client connected");

  char partHeader[64];
  unsigned long lastFrame = 0;
  unsigned long fpsStart = millis();
  int frames = 0;
  esp_err_t res = ESP_OK;

  while (res == ESP_OK) {
    // Limit the frame rate so Wi-Fi is not flooded
    unsigned long wait = 1000 / STREAM_MAX_FPS;
    if (millis() - lastFrame < wait) { delay(1); continue; }
    lastFrame = millis();

    camera_fb_t* fb; uint8_t* buf; size_t len;
    if (!grabJpeg(fb, buf, len)) { delay(10); continue; }

    size_t hLen = snprintf(partHeader, sizeof(partHeader), STREAM_PART, (unsigned)len);
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, partHeader, hLen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)buf, len);
    releaseJpeg(fb, buf);

    frames++;
    if (millis() - fpsStart >= 1000) {
      streamFps = frames * 1000.0f / (millis() - fpsStart);
      frames = 0;
      fpsStart = millis();
    }
  }
  streamFps = 0;
  Serial.println("[STREAM] Client disconnected");
  return res;
}

void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  httpd_uri_t root = { "/", HTTP_GET, rootHandler, nullptr };
  httpd_uri_t cap = { "/capture", HTTP_GET, captureHandler, nullptr };
  httpd_uri_t stat = { "/status", HTTP_GET, statusHandler, nullptr };
  httpd_uri_t flash = { "/flash", HTTP_GET, flashHandler, nullptr };
  if (httpd_start(&controlServer, &config) == ESP_OK) {
    httpd_register_uri_handler(controlServer, &root);
    httpd_register_uri_handler(controlServer, &cap);
    httpd_register_uri_handler(controlServer, &stat);
    httpd_register_uri_handler(controlServer, &flash);
  }

  config.server_port = 81;
  config.ctrl_port = 32769;
  httpd_uri_t stream = { "/stream", HTTP_GET, streamHandler, nullptr };
  if (httpd_start(&streamServer, &config) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &stream);
  }
}

// =============================================================================
// 4. SETUP & LOOP
// =============================================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
#if USE_STATIC_IP
  WiFi.config(IPAddress(STATIC_IP), IPAddress(STATIC_GATEWAY), IPAddress(STATIC_SUBNET), IPAddress(STATIC_GATEWAY));
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WIFI] Connecting to \"%s\"", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("[WIFI] Connected. Camera IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ECO ROVER ESP32-CAM ===");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  setupCamera();
  connectWiFi();
  if (MDNS.begin(MDNS_HOSTNAME)) MDNS.addService("http", "tcp", 80);
  startServers();

  Serial.printf("[READY] Stream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
}

void loop() {
  // Reconnect if Wi-Fi drops (the servers keep running and resume by themselves)
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WIFI] Lost connection, reconnecting...");
      WiFi.reconnect();
    }
  }
  delay(100);
}
