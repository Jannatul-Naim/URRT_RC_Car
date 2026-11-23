/*
 ===============================================================================
   ECO ROVER — ESP32 DEVKIT 2-WAY AUDIO BOARD (Arduino IDE sketch)

   What this board does (task: "Two-way audio acuity"):
     - LISTEN: records the INMP441 microphone and sends it to the laptop,
       so the operator hears the referee's 4-digit code.
     - TALK:   plays audio from the laptop on the MAX98357A amplifier +
       speaker, so the referee hears the operator.

   How it talks to the laptop (plain UDP, port 5005, no extra libraries):
     laptop -> board   "H"            hello; tells the board where to send mic audio
     laptop -> board   "A" + PCM      speaker audio
     board  -> laptop  "M" + PCM      microphone audio
   PCM = 16 kHz, 16-bit signed, mono, little-endian.

   Wiring (same as the electrical PDF, Sheet 7):
     INMP441 mic : SCK -> GPIO26, WS -> GPIO25, SD -> GPIO34, L/R -> GND, VDD -> 3V3
     MAX98357A   : BCLK -> GPIO27, LRC -> GPIO32, DIN -> GPIO33, SD -> GPIO23,
                   VIN -> 5V, GAIN open (9 dB)

   Board: "ESP32 Dev Module". Works with Arduino-ESP32 core 2.x and 3.x
   (core 3.x prints a "legacy I2S driver" warning; that is fine).
 ===============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <driver/i2s.h>

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
#define MDNS_HOSTNAME    "rover-audio"   // rover-audio.local

// Optional fixed IP so the laptop always finds this board at the same address
#define USE_STATIC_IP    false
#define STATIC_IP        192, 168, 1, 52
#define STATIC_GATEWAY   192, 168, 1, 1
#define STATIC_SUBNET    255, 255, 255, 0

#define UDP_PORT         5005
#define SAMPLE_RATE      16000

// Microphone (I2S port 0, receive)
#define MIC_PIN_SCK      26
#define MIC_PIN_WS       25
#define MIC_PIN_SD       34
#define MIC_SHIFT        14      // 32-bit sample -> 16-bit. Lower = louder (try 12..16)

// Speaker amplifier (I2S port 1, transmit)
#define SPK_PIN_BCLK     27
#define SPK_PIN_LRC      32
#define SPK_PIN_DIN      33
#define SPK_PIN_SD       23      // HIGH = amp on, LOW = amp off (no hiss when idle)
#define SPK_VOLUME       0.8f    // 0.0 .. 1.0
#define SPK_IDLE_OFF_MS  400     // switch the amp off after this much silence

#define MIC_CHUNK        256     // samples per UDP packet (16 ms)
#define LAPTOP_TIMEOUT_MS 3000   // stop sending mic audio if the laptop goes quiet

// =============================================================================
// 2. GLOBALS
// =============================================================================
WiFiUDP udp;       // receives from the laptop (loop)
WiFiUDP micUdp;    // sends mic audio (mic task), separate so the two tasks never share one
IPAddress laptopIp;
uint16_t laptopPort = 0;
volatile unsigned long lastHelloMs = 0;
unsigned long lastSpeakerMs = 0;
bool ampOn = false;

// =============================================================================
// 3. I2S SETUP
// =============================================================================
void setupMic() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;   // INMP441 sends 24 bits in a 32-bit slot
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;    // L/R pin tied to GND = left
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = MIC_CHUNK;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = MIC_PIN_SCK;
  pins.ws_io_num = MIC_PIN_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = MIC_PIN_SD;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
}

void setupSpeaker() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;   // we write every sample to both sides
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.tx_desc_auto_clear = true;                     // play silence when no data arrives

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = SPK_PIN_BCLK;
  pins.ws_io_num = SPK_PIN_LRC;
  pins.data_out_num = SPK_PIN_DIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pins);

  pinMode(SPK_PIN_SD, OUTPUT);
  digitalWrite(SPK_PIN_SD, LOW);   // amp off until audio arrives
}

// =============================================================================
// 4. MICROPHONE TASK: read the mic and send it to the laptop, forever
// =============================================================================
void micTask(void* arg) {
  static int32_t raw[MIC_CHUNK];
  static uint8_t packet[1 + MIC_CHUNK * 2];
  packet[0] = 'M';

  while (true) {
    size_t bytesRead = 0;
    i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytesRead, portMAX_DELAY);
    int n = bytesRead / 4;

    bool laptopAlive = laptopPort != 0 && (millis() - lastHelloMs) < LAPTOP_TIMEOUT_MS;
    if (!laptopAlive || n == 0) continue;

    for (int i = 0; i < n; i++) {
      int32_t s = raw[i] >> MIC_SHIFT;
      if (s > 32767) s = 32767;
      if (s < -32768) s = -32768;
      packet[1 + i * 2] = s & 0xFF;          // little-endian
      packet[2 + i * 2] = (s >> 8) & 0xFF;
    }
    micUdp.beginPacket(laptopIp, laptopPort);
    micUdp.write(packet, 1 + n * 2);
    micUdp.endPacket();
  }
}

// =============================================================================
// 5. SPEAKER: play one packet of PCM from the laptop
// =============================================================================
void playPcm(const uint8_t* data, int bytes) {
  static int16_t stereo[1024];               // L,R pairs
  int samples = bytes / 2;
  if (samples > 512) samples = 512;

  for (int i = 0; i < samples; i++) {
    int16_t s = (int16_t)(data[i * 2] | (data[i * 2 + 1] << 8));
    s = (int16_t)(s * SPK_VOLUME);
    stereo[i * 2] = s;
    stereo[i * 2 + 1] = s;
  }
  if (!ampOn) {
    digitalWrite(SPK_PIN_SD, HIGH);
    ampOn = true;
  }
  size_t written = 0;
  i2s_write(I2S_NUM_1, stereo, samples * 4, &written, pdMS_TO_TICKS(100));
  lastSpeakerMs = millis();
}

// =============================================================================
// 6. SETUP & LOOP
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
  Serial.printf("[WIFI] Connected. Audio board IP: %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ECO ROVER AUDIO BOARD ===");

  setupMic();
  setupSpeaker();
  connectWiFi();
  if (MDNS.begin(MDNS_HOSTNAME)) Serial.println("[mDNS] rover-audio.local");

  udp.begin(UDP_PORT);
  xTaskCreatePinnedToCore(micTask, "mic", 8192, NULL, 2, NULL, 1);
  Serial.printf("[READY] Listening for the laptop on UDP port %d\n", UDP_PORT);
}

void loop() {
  static uint8_t buf[1 + 1024];
  int len = udp.parsePacket();
  if (len > 0) {
    len = udp.read(buf, sizeof(buf));
    if (len >= 1 && buf[0] == 'H') {
      if (laptopPort == 0 || udp.remoteIP() != laptopIp) {
        Serial.printf("[LINK] Laptop at %s\n", udp.remoteIP().toString().c_str());
      }
      laptopIp = udp.remoteIP();
      laptopPort = udp.remotePort();
      lastHelloMs = millis();
    } else if (len > 1 && buf[0] == 'A') {
      playPcm(buf + 1, len - 1);
    }
  }

  // Switch the amplifier off when the operator stops talking (removes hiss)
  if (ampOn && millis() - lastSpeakerMs > SPK_IDLE_OFF_MS) {
    digitalWrite(SPK_PIN_SD, LOW);
    ampOn = false;
  }

  // Reconnect if Wi-Fi drops
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) {
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }

  if (len <= 0) delay(1);
}
