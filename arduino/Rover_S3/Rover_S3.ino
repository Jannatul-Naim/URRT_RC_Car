/*
 ===============================================================================
   ECO ROVER — ESP32-S3 MAIN CONTROLLER (Arduino IDE sketch)

   What this board does:
     - Drives the 4 motors through 2x BTS7960 (IBT-2)
     - Reads the 4 HC-SR04 sonars (proximity task)
     - Moves the camera pan servo (MG996R) on GPIO 8
     - Reads the compass (QMC5883P, I2C on GPIO 39/40) for heading
     - Pings the IP address decoded from the QR code (rule 2.1.3: the ping
       must come from the rover, so the S3 sends it, not the laptop)
     - Reads the motor-bus voltage on GPIO 2: no battery or E-stop OFF = ESTOP

   The dashboard no longer lives here. It runs on the laptop
   (laptop/server.py), which connects to this board on ws://<S3-IP>:81.

   Pins:
     - Left Motor Forward  : GPIO 4 (RPWM)     - Right Motor Forward : GPIO 6 (RPWM)
     - Left Motor Reverse  : GPIO 5 (LPWM)     - Right Motor Reverse : GPIO 7 (LPWM)
     - Driver Enable (EN)  : tied to each driver's own VCC (3.3 V), always enabled
     - Pan servo signal    : GPIO 8 (MG996R)
     - Compass I2C         : SDA GPIO 39, SCL GPIO 40 (QMC5883P, 3.3 V)
     - Sonars Trig/Echo    : 10/11, 12/13, 14/16, 17/18 (Echo via 1k/1.8k divider)
     - Motor bus sense     : GPIO 2 (100k/22k divider, R23/R24, required)

   WebSocket text commands (port 81), one per message:
     D,<left -100..100>,<right -100..100>,<pwm cap 0..100>   drive
     S,<angle 0..180>                                         pan servo
     P,<ipv4>                                                 ping that address
     CMD:ESTOP / CMD:RESET                                    software E-stop
     PING                                                     answers PONG
   The S3 sends back:
     T,<lPct>,<rPct>,<lRaw>,<rRaw>,<busV>,<estop>,<s1>,<s2>,<s3>,<s4>,<servo>,<heading>
       heading = compass degrees 0..359, -1 when no compass is found
     L,<level>,<text>                                         log line
     G,<ip>,<ok 0/1>,<rtt ms>                                 ping result

   Board: "ESP32S3 Dev Module". Works with Arduino-ESP32 core 2.x and 3.x.
 ===============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "ping/ping_sock.h"

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

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
#define WIFI_CONNECT_TIMEOUT  12000      // 12s timeout before fallback to AP

#define AP_SSID               "EcoEco-Rover-AP"   // AP_PASSWORD is in secrets.h
#define AP_IP                 192, 168, 4, 1
#define MDNS_HOSTNAME         "rover"    // Access at http://rover.local

// Optional fixed IP, so the laptop always finds the S3 at the same address.
// Leave false to use DHCP (then read the IP from the Serial Monitor).
#define USE_STATIC_IP         false
#define STATIC_IP             192, 168, 1, 50
#define STATIC_GATEWAY        192, 168, 1, 1
#define STATIC_SUBNET         255, 255, 255, 0

#define HTTP_PORT             80
#define WS_PORT               81         // Dedicated WebSocket port

// 4 MOTOR PWM PINS ONLY (EN is tied to each driver's VCC, 3.3 V, in the circuit)
#define PIN_LEFT_RPWM         4          // GPIO4: Left Driver Forward PWM
#define PIN_LEFT_LPWM         5          // GPIO5: Left Driver Reverse PWM
#define PIN_RIGHT_RPWM        6          // GPIO6: Right Driver Forward PWM
#define PIN_RIGHT_LPWM        7          // GPIO7: Right Driver Reverse PWM

// 4 ULTRASONIC SENSORS (HC-SR04 x4) — Safe Logic Level via 1k/1.8k Dividers
#define PIN_SONAR1_TRIG       10         // Front Centre Trig (GPIO 10)
#define PIN_SONAR1_ECHO       11         // Front Centre Echo (GPIO 11 via R11 1k/1.8k)
#define PIN_SONAR2_TRIG       12         // Front Left Trig (GPIO 12)
#define PIN_SONAR2_ECHO       13         // Front Left Echo (GPIO 13 via R13 1k/1.8k)
#define PIN_SONAR3_TRIG       14         // Front Right Trig (GPIO 14)
#define PIN_SONAR3_ECHO       16         // Front Right Echo (GPIO 16 via R15 1k/1.8k)
#define PIN_SONAR4_TRIG       17         // Rear Trig (GPIO 17)
#define PIN_SONAR4_ECHO       18         // Rear Echo (GPIO 18 via R17 1k/1.8k)

#define SONAR_COUNT           4
#define SONAR_MAX_DIST_CM     250        // Max measurement range (cm)
#define SONAR_TIMEOUT_US      14500      // 14.5ms max pulseIn wait (~250cm range)
#define SONAR_PING_INTERVAL_MS 35        // Non-blocking round-robin ping (sweep all 4 in 140ms)
#define OBSTACLE_ALERT_DIST_CM 20        // Close obstacle warning threshold (cm)

#define MOTOR_LEFT_INVERT     false
#define MOTOR_RIGHT_INVERT    false

// ANTI-BURN TURNING SAFETY PARAMETERS
#define PWM_FREQUENCY_HZ      20000      // 20 kHz ultrasonic PWM (smooth & silent)
#define PWM_RESOLUTION_BITS   8          // 8-bit resolution (0 to 255)
#define PWM_MAX_RAW           255
#define MOTOR_SLEW_STEP       15         // Max PWM change per 10ms cycle (prevents back-EMF spikes)
#define MOTOR_DEADBAND_RAW    10         // Below this duty, output 0 to prevent motor coil buzz
#define SPIN_TURN_DERATE      0.80f      // 20% power derating during pivot turns to prevent stall heat
#define DIRECTION_DEADTIME_MS 15         // Deadtime before reversing H-bridge direction
#define WATCHDOG_TIMEOUT_MS   450        // Coasts to stop if connection drops >450ms

// CAMERA PAN SERVO (MG996R). Signal wire on GPIO 8. Power it from the servo
// buck U7 (+5V SERVO), never from the S3's 5V pin: an MG996R can draw up to
// about 2.5 A when it stalls. Use the 180-degree type, not a 360 (continuous).
#define PIN_SERVO             8
#define SERVO_MIN_DEG         0          // narrow these if the camera cable pulls
#define SERVO_MAX_DEG         180
#define SERVO_CENTER_DEG      90
#define SERVO_MIN_US          500        // pulse width at 0 degrees
#define SERVO_MAX_US          2400       // pulse width at 180 degrees

// MOTOR BUS VOLTAGE SENSE = HARDWARE E-STOP SENSE. R23 (100k) from the motor
// bus and R24 (22k) to GND meet at GPIO 2. If the motor battery is unplugged
// or the E-stop switch is OFF, the bus reads 0 V and the rover latches ESTOP.
// R23/R24 MUST be fitted: without them GPIO 2 reads 0 V and the rover stays in
// ESTOP. For a bench test with no divider only, set this to false.
#define USE_BUS_SENSE         true
#define PIN_BUS_SENSE         2
#define BUS_DIVIDER_RATIO     (122.0f / 22.0f)   // (R23 + R24) / R24
#define BUS_ESTOP_BELOW_V     6.0f       // below this: no battery or E-stop OFF

// COMPASS (QMC5883P; the GY-271 board is sold as "HMC5883L"). VCC to the S3
// 3V3 pin, never 5 V. Get the offsets by turning the rover through a full
// circle with arduino/Compass_Test and copying its "offset X / Y" numbers.
#define PIN_COMPASS_SDA       39
#define PIN_COMPASS_SCL       40
#define COMPASS_OFFSET_X      0          // hard-iron offset, from Compass_Test
#define COMPASS_OFFSET_Y      0
#define COMPASS_DECLINATION_DEG 0.0f     // your local magnetic declination, east +
#define COMPASS_MOUNT_DEG     0.0f       // add if 0 deg is not the rover's front

// =============================================================================
// 3. LOGGING BUFFER (Streams Serial Monitor events to Dashboard Terminal)
// =============================================================================
#define MAX_LOG_LINES 32
struct LogEntry {
  uint32_t id;
  char text[128];
  char level[8]; // "sys", "info", "warn", "err"
};

LogEntry logBuffer[MAX_LOG_LINES];
int logHead = 0;
uint32_t nextLogId = 1;

class RoverWebSocketServer;
extern RoverWebSocketServer wsServer;

void roverLog(const char* level, const char* msg);

// =============================================================================
// 4. SELF-CONTAINED CRYPTO (SHA-1 & Base64 for RFC6455 WebSocket)
// =============================================================================
namespace TinyCrypto {
  static uint32_t rol(uint32_t value, size_t bits) { return (value << bits) | (value >> (32 - bits)); }
  static void sha1(const uint8_t* data, size_t len, uint8_t hash[20]) {
    uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0;
    // 64-bit on purpose: size_t is 32 bits on the ESP32, and shifting it by
    // 32..56 below wrapped around, wrote a wrong length into the padding and
    // made every WebSocket handshake fail ("Invalid challenge response").
    uint64_t totalBits = (uint64_t)len * 8;
    size_t paddedLen = len + 1 + 8;
    while (paddedLen % 64 != 0) paddedLen++;
    uint8_t* p = (uint8_t*)malloc(paddedLen);
    if (!p) return;
    memset(p, 0, paddedLen);
    memcpy(p, data, len);
    p[len] = 0x80;
    for (int i = 0; i < 8; i++) p[paddedLen - 1 - i] = (uint8_t)((totalBits >> (i * 8)) & 0xFF);
    for (size_t chunk = 0; chunk < paddedLen; chunk += 64) {
      uint32_t w[80];
      for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)p[chunk + i * 4] << 24) | ((uint32_t)p[chunk + i * 4 + 1] << 16) |
               ((uint32_t)p[chunk + i * 4 + 2] << 8) | ((uint32_t)p[chunk + i * 4 + 3]);
      }
      for (int i = 16; i < 80; i++) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
      uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
      for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20) { f = (b & c) | ((~b) & d); k = 0x5A827999; }
        else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
        else { f = b ^ c ^ d; k = 0xCA62C1D6; }
        uint32_t temp = rol(a, 5) + f + e + k + w[i];
        e = d; d = c; c = rol(b, 30); b = a; a = temp;
      }
      h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }
    free(p);
    uint32_t h[5] = { h0, h1, h2, h3, h4 };
    for (int i = 0; i < 5; i++) {
      hash[i * 4]     = (uint8_t)(h[i] >> 24);
      hash[i * 4 + 1] = (uint8_t)(h[i] >> 16);
      hash[i * 4 + 2] = (uint8_t)(h[i] >> 8);
      hash[i * 4 + 3] = (uint8_t)(h[i]);
    }
  }
  static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  static String base64_encode(const uint8_t* in, size_t len) {
    String out = "";
    int val = 0, valb = -6;
    for (size_t i = 0; i < len; i++) {
      val = (val << 8) + in[i]; valb += 8;
      while (valb >= 0) { out += b64_table[(val >> valb) & 0x3F]; valb -= 6; }
    }
    if (valb > -6) out += b64_table[((val << 8) >> (valb + 8)) & 0x3F];
    while (out.length() % 4) out += '=';
    return out;
  }
}

// =============================================================================
// 5. BTS7960 MOTOR CONTROLLER CLASS (With Anti-Burn Turning Slew Limiter)
// =============================================================================
#define CH_LEFT_RPWM   0
#define CH_LEFT_LPWM   1
#define CH_RIGHT_RPWM  2
#define CH_RIGHT_LPWM  3

static void ledc_init_pin(int pin, int channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
  ledcSetup(channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(pin, channel);
#endif
}

static void ledc_write_duty(int pin, int channel, int duty) {
  duty = constrain(duty, 0, PWM_MAX_RAW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

class MotorController {
public:
  MotorController()
    : _targetLeftRaw(0), _targetRightRaw(0), _currLeftRaw(0), _currRightRaw(0),
      _pwmLimit(75), _lastCmdTime(0), _lastUpdateTime(0), _leftZeroTime(0), _rightZeroTime(0),
      _estopLatched(false), _watchdogTripped(false) {}

  void begin() {
    pinMode(PIN_LEFT_RPWM, OUTPUT); digitalWrite(PIN_LEFT_RPWM, LOW);
    pinMode(PIN_LEFT_LPWM, OUTPUT); digitalWrite(PIN_LEFT_LPWM, LOW);
    pinMode(PIN_RIGHT_RPWM, OUTPUT); digitalWrite(PIN_RIGHT_RPWM, LOW);
    pinMode(PIN_RIGHT_LPWM, OUTPUT); digitalWrite(PIN_RIGHT_LPWM, LOW);

    ledc_init_pin(PIN_LEFT_RPWM, CH_LEFT_RPWM);
    ledc_init_pin(PIN_LEFT_LPWM, CH_LEFT_LPWM);
    ledc_init_pin(PIN_RIGHT_RPWM, CH_RIGHT_RPWM);
    ledc_init_pin(PIN_RIGHT_LPWM, CH_RIGHT_LPWM);

    _lastCmdTime = millis();
    _lastUpdateTime = millis();
    roverLog("sys", "[MOTORS] BTS7960 4-Pin Driver Armed: L(GPIO 4,5), R(GPIO 6,7)");
  }

  void setDrive(int leftPct, int rightPct, int pwmLimit) {
    if (_estopLatched) { _targetLeftRaw = 0; _targetRightRaw = 0; return; }
    _lastCmdTime = millis();
    _watchdogTripped = false;

    _pwmLimit = constrain(pwmLimit, 0, 100);
    float capFactor = (float)_pwmLimit / 100.0f;
    leftPct = constrain(leftPct, -100, 100);
    rightPct = constrain(rightPct, -100, 100);

    if (MOTOR_LEFT_INVERT) leftPct = -leftPct;
    if (MOTOR_RIGHT_INVERT) rightPct = -rightPct;

    // TURNING PROTECTION: Derate power during spin turns to prevent 25GA motor overheating
    bool isSpinTurn = (leftPct * rightPct < 0);
    float turnMult = isSpinTurn ? SPIN_TURN_DERATE : 1.0f;

    float scaledL = (float)leftPct * capFactor * turnMult;
    float scaledR = (float)rightPct * capFactor * turnMult;

    _targetLeftRaw = (int)round((scaledL / 100.0f) * (float)PWM_MAX_RAW);
    _targetRightRaw = (int)round((scaledR / 100.0f) * (float)PWM_MAX_RAW);
  }

  void update() {
    unsigned long now = millis();
    if (now - _lastUpdateTime < 10) return; // 100 Hz loop
    _lastUpdateTime = now;

    if (_estopLatched) { _currLeftRaw = 0; _currRightRaw = 0; applyHardwarePwm(0, 0); return; }

    // Failsafe watchdog
    if ((now - _lastCmdTime) > WATCHDOG_TIMEOUT_MS) {
      if (!_watchdogTripped) {
        _watchdogTripped = true;
        roverLog("warn", "[WATCHDOG] Inactive >450ms -> Motors stopped.");
      }
      _targetLeftRaw = 0;
      _targetRightRaw = 0;
    }

    // Left Motor Ramping & Deadtime
    if (_targetLeftRaw != _currLeftRaw) {
      if ((_currLeftRaw > 0 && _targetLeftRaw < 0) || (_currLeftRaw < 0 && _targetLeftRaw > 0)) {
        if (_currLeftRaw > 0) _currLeftRaw = max(0, _currLeftRaw - MOTOR_SLEW_STEP);
        else _currLeftRaw = min(0, _currLeftRaw + MOTOR_SLEW_STEP);
        if (_currLeftRaw == 0) _leftZeroTime = now;
      } else {
        if (_currLeftRaw == 0 && (now - _leftZeroTime) < DIRECTION_DEADTIME_MS) { /* deadtime hold */ }
        else {
          int d = constrain(_targetLeftRaw - _currLeftRaw, -MOTOR_SLEW_STEP, MOTOR_SLEW_STEP);
          _currLeftRaw += d;
        }
      }
    }

    // Right Motor Ramping & Deadtime
    if (_targetRightRaw != _currRightRaw) {
      if ((_currRightRaw > 0 && _targetRightRaw < 0) || (_currRightRaw < 0 && _targetRightRaw > 0)) {
        if (_currRightRaw > 0) _currRightRaw = max(0, _currRightRaw - MOTOR_SLEW_STEP);
        else _currRightRaw = min(0, _currRightRaw + MOTOR_SLEW_STEP);
        if (_currRightRaw == 0) _rightZeroTime = now;
      } else {
        if (_currRightRaw == 0 && (now - _rightZeroTime) < DIRECTION_DEADTIME_MS) { /* deadtime hold */ }
        else {
          int d = constrain(_targetRightRaw - _currRightRaw, -MOTOR_SLEW_STEP, MOTOR_SLEW_STEP);
          _currRightRaw += d;
        }
      }
    }

    int outL = (abs(_currLeftRaw) < MOTOR_DEADBAND_RAW) ? 0 : _currLeftRaw;
    int outR = (abs(_currRightRaw) < MOTOR_DEADBAND_RAW) ? 0 : _currRightRaw;
    applyHardwarePwm(outL, outR);
  }

  void emergencyStop(const char* reason = "E-STOP") {
    _estopLatched = true;
    _targetLeftRaw = 0; _targetRightRaw = 0;
    _currLeftRaw = 0; _currRightRaw = 0;
    applyHardwarePwm(0, 0);
    char buf[64];
    snprintf(buf, sizeof(buf), "[E-STOP] %s -> 0 PWM", reason);
    roverLog("err", buf);
  }

  void resetEmergencyStop() {
    _estopLatched = false; _watchdogTripped = false;
    _targetLeftRaw = 0; _targetRightRaw = 0;
    _currLeftRaw = 0; _currRightRaw = 0;
    _lastCmdTime = millis();
    applyHardwarePwm(0, 0);
    roverLog("sys", "[E-STOP] Cleared and Re-Armed.");
  }

  int getActualLeftPct() const { return (int)round(((float)_currLeftRaw / 255.0f) * 100.0f); }
  int getActualRightPct() const { return (int)round(((float)_currRightRaw / 255.0f) * 100.0f); }
  int getActualLeftRaw() const { return _currLeftRaw; }
  int getActualRightRaw() const { return _currRightRaw; }
  int getPwmCap() const { return _pwmLimit; }
  bool isEmergencyStopped() const { return _estopLatched; }

private:
  int _targetLeftRaw, _targetRightRaw, _currLeftRaw, _currRightRaw, _pwmLimit;
  unsigned long _lastCmdTime, _lastUpdateTime, _leftZeroTime, _rightZeroTime;
  bool _estopLatched, _watchdogTripped;

  void applyHardwarePwm(int leftRaw, int rightRaw) {
    writeMotorPins(PIN_LEFT_RPWM, PIN_LEFT_LPWM, leftRaw);
    writeMotorPins(PIN_RIGHT_RPWM, PIN_RIGHT_LPWM, rightRaw);
  }

  void writeMotorPins(int pinRPWM, int pinLPWM, int rawVal) {
    int chRPWM = (pinRPWM == PIN_LEFT_RPWM) ? CH_LEFT_RPWM : CH_RIGHT_RPWM;
    int chLPWM = (pinLPWM == PIN_LEFT_LPWM) ? CH_LEFT_LPWM : CH_RIGHT_LPWM;
    if (rawVal > 0) {
      ledc_write_duty(pinLPWM, chLPWM, 0);
      ledc_write_duty(pinRPWM, chRPWM, rawVal);
    } else if (rawVal < 0) {
      ledc_write_duty(pinRPWM, chRPWM, 0);
      ledc_write_duty(pinLPWM, chLPWM, abs(rawVal));
    } else {
      ledc_write_duty(pinRPWM, chRPWM, 0);
      ledc_write_duty(pinLPWM, chLPWM, 0);
    }
  }
};

MotorController motors;

// =============================================================================
// 6. ULTRASONIC SENSOR MANAGER (Non-Blocking Round-Robin HC-SR04 Driver)
// =============================================================================
class SonarManager {
public:
  SonarManager() : _currentSensor(0), _lastPingTime(0) {
    for (int i = 0; i < SONAR_COUNT; i++) {
      _distances[i] = -1;
      _lastAlertState[i] = false;
    }
    _trigPins[0] = PIN_SONAR1_TRIG; _echoPins[0] = PIN_SONAR1_ECHO;
    _trigPins[1] = PIN_SONAR2_TRIG; _echoPins[1] = PIN_SONAR2_ECHO;
    _trigPins[2] = PIN_SONAR3_TRIG; _echoPins[2] = PIN_SONAR3_ECHO;
    _trigPins[3] = PIN_SONAR4_TRIG; _echoPins[3] = PIN_SONAR4_ECHO;
  }

  void begin() {
    for (int i = 0; i < SONAR_COUNT; i++) {
      pinMode(_trigPins[i], OUTPUT);
      digitalWrite(_trigPins[i], LOW);
      // Pull-down: an unwired or broken echo line reads LOW (no echo, "CLEAR")
      // instead of floating and giving random distances and fake alerts.
      pinMode(_echoPins[i], INPUT_PULLDOWN);
    }
  }

  void update() {
    unsigned long now = millis();
    if (now - _lastPingTime >= SONAR_PING_INTERVAL_MS) {
      _lastPingTime = now;
      measureSensor(_currentSensor);
      _currentSensor = (_currentSensor + 1) % SONAR_COUNT;
    }
  }

  int getDistance(int index) const {
    if (index >= 0 && index < SONAR_COUNT) {
      return _distances[index];
    }
    return -1;
  }

private:
  uint8_t _trigPins[SONAR_COUNT];
  uint8_t _echoPins[SONAR_COUNT];
  int _distances[SONAR_COUNT];
  bool _lastAlertState[SONAR_COUNT];
  int _currentSensor;
  unsigned long _lastPingTime;

  void measureSensor(int idx) {
    uint8_t trig = _trigPins[idx];
    uint8_t echo = _echoPins[idx];

    // 10us HIGH pulse
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // Read echo pulse with timeout (max ~14.5 ms)
    unsigned long duration = pulseIn(echo, HIGH, SONAR_TIMEOUT_US);
    int dist = -1;
    if (duration > 0) {
      dist = (int)(duration / 58);
      if (dist < 2 || dist > SONAR_MAX_DIST_CM) {
        dist = -1;
      }
    }
    _distances[idx] = dist;

    // Check obstacle close alert (< 20 cm)
    bool isClose = (dist > 0 && dist <= OBSTACLE_ALERT_DIST_CM);
    if (isClose && !_lastAlertState[idx]) {
      _lastAlertState[idx] = true;
      char alertBuf[80];
      const char* sName = (idx == 0) ? "Sonar_1 (Center)" :
                          (idx == 1) ? "Sonar_2 (Left)" :
                          (idx == 2) ? "Sonar_3 (Right)" : "Sonar_4 (Rear)";
      snprintf(alertBuf, sizeof(alertBuf), "[ALERT] %s OBSTACLE CLOSE: %d cm!", sName, dist);
      roverLog("warn", alertBuf);
    } else if (!isClose && _lastAlertState[idx]) {
      _lastAlertState[idx] = false;
    }
  }
};

SonarManager sonars;

// =============================================================================
// 6b. CAMERA PAN SERVO (MG996R on GPIO 8, 50 Hz LEDC PWM)
// =============================================================================
#define SERVO_LEDC_CHANNEL  4      // channels 0-3 are the motors (core 2.x only)
#define SERVO_FREQ_HZ       50
#define SERVO_RES_BITS      14     // 0..16383 per 20 ms period

class PanServo {
public:
  void begin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_SERVO, SERVO_FREQ_HZ, SERVO_RES_BITS);
#else
    ledcSetup(SERVO_LEDC_CHANNEL, SERVO_FREQ_HZ, SERVO_RES_BITS);
    ledcAttachPin(PIN_SERVO, SERVO_LEDC_CHANNEL);
#endif
    write(SERVO_CENTER_DEG);
  }

  void write(int deg) {
    _angle = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    long us = map(_angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    uint32_t duty = (uint32_t)((us * ((1UL << SERVO_RES_BITS) - 1)) / 20000L);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_SERVO, duty);
#else
    ledcWrite(SERVO_LEDC_CHANNEL, duty);
#endif
  }

  int angle() const { return _angle; }

private:
  int _angle = SERVO_CENTER_DEG;
};

PanServo panServo;

// =============================================================================
// 6c. MOTOR BUS VOLTAGE (optional battery level + hardware E-stop switch sense)
// =============================================================================
float busVolts = -1.0f;   // -1 = not measured (USE_BUS_SENSE is false)

void readBusVoltage() {
#if USE_BUS_SENSE
  static unsigned long last = 0;
  if (millis() - last < 10) return;   // every 10 ms
  last = millis();
  float v = (analogReadMilliVolts(PIN_BUS_SENSE) / 1000.0f) * BUS_DIVIDER_RATIO;
  busVolts = (busVolts < 0) ? v : busVolts * 0.8f + v * 0.2f;   // light smoothing
#endif
}

bool busIsOff() {
#if USE_BUS_SENSE
  return busVolts >= 0 && busVolts < BUS_ESTOP_BELOW_V;
#else
  return false;
#endif
}

// =============================================================================
// 6c2. COMPASS (QMC5883P on I2C, SDA GPIO 39 / SCL GPIO 40)
// =============================================================================
class Compass {
public:
  void begin() {
    Wire.begin(PIN_COMPASS_SDA, PIN_COMPASS_SCL, 100000);
    Wire.setTimeOut(5);   // ms: a loose wire must not stall the motor loop
    uint8_t id = 0;
    _ok = readRegs(0x00, 1, &id) && id == 0x80;   // QMC5883P chip ID
    if (_ok) {
      writeReg(0x29, 0x06);   // axis sign setup (datasheet)
      writeReg(0x0B, 0x08);   // SET/RESET on, +-8 G
      writeReg(0x0A, 0xCD);   // normal mode, 200 Hz, OSR 8/8
    }
  }

  void update() {
    if (!_ok || millis() - _last < 50) return;   // 20 readings a second
    _last = millis();
    uint8_t d[6];
    if (!readRegs(0x01, 6, d)) {
      if (++_fails >= 10) _heading = -1;          // lost for half a second
      return;
    }
    _fails = 0;
    int16_t x = (int16_t)(d[0] | (d[1] << 8));
    int16_t y = (int16_t)(d[2] | (d[3] << 8));
    float h = atan2f((float)(y - COMPASS_OFFSET_Y), (float)(x - COMPASS_OFFSET_X)) * 180.0f / PI
              + COMPASS_DECLINATION_DEG + COMPASS_MOUNT_DEG;
    while (h < 0) h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    _heading = (int)(h + 0.5f) % 360;
  }

  bool present() const { return _ok; }
  int heading() const { return _heading; }   // -1 = no reading

private:
  static const uint8_t ADDR = 0x2C;
  bool _ok = false;
  int _heading = -1;
  uint8_t _fails = 0;
  unsigned long _last = 0;

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }
  bool readRegs(uint8_t reg, uint8_t n, uint8_t* out) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADDR, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) out[i] = Wire.read();
    return true;
  }
};

Compass compass;

// =============================================================================
// 6d. ICMP PING (QR task: ping the IP from the QR code, from the rover itself)
// =============================================================================
volatile bool pingBusy = false;
volatile bool pingDone = false;
volatile uint32_t pingReplies = 0;
volatile uint32_t pingRttMs = 0;
char pingTarget[20] = "";

static void onPingSuccess(esp_ping_handle_t hdl, void* arg) {
  uint32_t t = 0;
  esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &t, sizeof(t));
  pingRttMs = t;
  pingReplies++;
}

static void onPingEnd(esp_ping_handle_t hdl, void* arg) {
  esp_ping_delete_session(hdl);
  pingBusy = false;
  pingDone = true;   // loop() reports the result
}

// Sends 4 echo requests to ip. Returns false if busy or the address is bad.
bool startPing(const String& ip) {
  if (pingBusy) return false;
  ip_addr_t target;
  if (!ipaddr_aton(ip.c_str(), &target)) return false;

  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.target_addr = target;
  cfg.count = 4;
  cfg.interval_ms = 500;
  cfg.timeout_ms = 1000;

  esp_ping_callbacks_t cbs = {};
  cbs.on_ping_success = onPingSuccess;
  cbs.on_ping_end = onPingEnd;

  esp_ping_handle_t handle = nullptr;
  if (esp_ping_new_session(&cfg, &cbs, &handle) != ESP_OK) return false;
  strncpy(pingTarget, ip.c_str(), sizeof(pingTarget) - 1);
  pingReplies = 0;
  pingRttMs = 0;
  pingBusy = true;
  esp_ping_start(handle);
  return true;
}

// =============================================================================
// 7. EMBEDDED DUAL PROTOCOL: HIGH-SPEED WEBSOCKET (Port 81) & REST API (Port 80)
// =============================================================================
class RoverWebSocketServer {
public:
  RoverWebSocketServer(uint16_t port = WS_PORT) 
    : _server(port), _handshakeDone(false), _port(port), _handshakeStartTime(0), _handshakeBuffer("") {}

  void begin() {
    _server.begin();
    _server.setNoDelay(true);
    roverLog("sys", String("[WS] WebSocket Server listening on port " + String(_port)).c_str());
  }

  void update() {
    if (_server.hasClient()) {
      WiFiClient incoming = _server.available();
      if (incoming) {
        if (!bool(_client) || (_client.connected() == 0)) {
          _client = incoming;
          _client.setNoDelay(true);
          _handshakeDone = false;
          _handshakeBuffer = "";
          _handshakeStartTime = millis();
          roverLog("info", "[WS] New client connected on port 81");
        } else {
          incoming.stop();
        }
      }
    }

    if (!bool(_client) || (_client.connected() == 0)) {
      _handshakeDone = false;
      return;
    }

    if (!_handshakeDone) {
      while (_client.available() > 0) {
        char c = (char)_client.read();
        _handshakeBuffer += c;
        if (_handshakeBuffer.endsWith("\r\n\r\n")) {
          if (doHandshake(_handshakeBuffer)) {
            _handshakeDone = true;
            _handshakeBuffer = "";
            roverLog("sys", "[WS] Control WebSocket connected on port 81! S3 Ready.");
          } else {
            roverLog("warn", "[WS] WebSocket handshake failed.");
            _client.stop();
            _handshakeDone = false;
            _handshakeBuffer = "";
          }
          break;
        }
      }
      if (!_handshakeDone && (millis() - _handshakeStartTime > 3000)) {
        roverLog("warn", "[WS] Handshake timeout.");
        _client.stop();
        _handshakeDone = false;
        _handshakeBuffer = "";
      }
      return;
    }

    while (_client.available() >= 2) {
      processFrame();
    }
  }

  void sendTelemetry(int leftPct, int rightPct, int leftRaw, int rightRaw, int s1 = -1, int s2 = -1, int s3 = -1, int s4 = -1) {
    char buf[112];
    snprintf(buf, sizeof(buf), "T,%d,%d,%d,%d,%.1f,%d,%d,%d,%d,%d,%d,%d",
             leftPct, rightPct, leftRaw, rightRaw, busVolts,
             motors.isEmergencyStopped() ? 1 : 0,
             s1, s2, s3, s4, panServo.angle(), compass.heading());
    sendText(buf);
  }

  void sendLine(const char* text) { sendText(text); }

  void logToDashboard(const char* level, const char* text) {
    char buf[256];
    snprintf(buf, sizeof(buf), "L,%s,%s", level ? level : "info", text ? text : "");
    sendText(buf);
  }

  bool hasClient() { return _handshakeDone && bool(_client) && (_client.connected() != 0); }

private:
  WiFiServer _server;
  WiFiClient _client;
  bool _handshakeDone;
  uint16_t _port;
  unsigned long _handshakeStartTime;
  String _handshakeBuffer;

  bool doHandshake(const String& req) {
    String reqLower = req;
    reqLower.toLowerCase();
    int idx = reqLower.indexOf("sec-websocket-key:");
    if (idx == -1) return false;
    int lineEnd = req.indexOf("\r\n", idx);
    if (lineEnd == -1) lineEnd = req.indexOf("\n", idx);
    if (lineEnd == -1) return false;

    String key = req.substring(idx + 18, lineEnd);
    key.trim();
    if (key.length() == 0) return false;

    String combined = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    uint8_t hash[20];
    TinyCrypto::sha1((const uint8_t*)combined.c_str(), combined.length(), hash);
    String acceptKey = TinyCrypto::base64_encode(hash, 20);

    String resp = "HTTP/1.1 101 Switching Protocols\r\n"
                  "Upgrade: websocket\r\n"
                  "Connection: Upgrade\r\n"
                  "Sec-WebSocket-Accept: " + acceptKey + "\r\n\r\n";
    _client.print(resp);
    // No _client.flush() here: on the ESP32, WiFiClient::flush() throws away
    // received bytes, so it could eat the laptop's first command.
    return true;
  }

  void processFrame() {
    uint8_t b0 = _client.read(), b1 = _client.read();
    uint8_t opcode = b0 & 0x0F;
    bool isMasked = (b1 & 0x80) != 0;
    uint64_t len = b1 & 0x7F;

    if (len == 126) {
      unsigned long w = millis();
      while (_client.available() < 2 && _client.connected() && (millis() - w < 50)) delay(1);
      if (_client.available() < 2) return;
      len = ((uint16_t)_client.read() << 8) | _client.read();
    } else if (len == 127) {
      _client.stop();
      _handshakeDone = false;
      return;
    }

    uint8_t mask[4] = {0};
    if (isMasked) {
      unsigned long w = millis();
      while (_client.available() < 4 && _client.connected() && (millis() - w < 50)) delay(1);
      if (_client.available() < 4) return;
      for (int i = 0; i < 4; i++) mask[i] = _client.read();
    }

    String msg = "";
    msg.reserve((size_t)len + 1);
    unsigned long waitStart = millis();
    while ((size_t)msg.length() < len && _client.connected() && (millis() - waitStart < 50)) {
      while (_client.available() > 0 && (size_t)msg.length() < len) {
        size_t i = msg.length();
        uint8_t c = _client.read();
        if (isMasked) c ^= mask[i % 4];
        msg += (char)c;
      }
      if ((size_t)msg.length() < len) delay(1);
    }
    if ((size_t)msg.length() < len) {
      // Half a frame: the rest would be read as a new header and garble the
      // stream. Drop the link; the laptop reconnects within a second.
      roverLog("warn", "[WS] Incomplete frame, reconnecting");
      _client.stop();
      _handshakeDone = false;
      return;
    }

    if (opcode == 0x01) { // Text
      if (msg.startsWith("D,")) {
        int c1 = msg.indexOf(',', 2);
        int c2 = msg.indexOf(',', c1 + 1);
        int c3 = msg.indexOf(',', c2 + 1);
        if (c1 != -1 && c2 != -1) {
          int l = msg.substring(2, c1).toInt();
          int r = msg.substring(c1 + 1, c2).toInt();
          // The cap is the 3rd number: "D,l,r,pwm" (laptop /api/drive) or
          // "D,l,r,pwm,0" (dashboard pages). Before, the 4-field form fell back
          // to 75 % and ignored the cap. A missing cap now means 0 (no drive).
          String capText = (c3 != -1) ? msg.substring(c2 + 1, c3) : msg.substring(c2 + 1);
          capText.trim();
          int p = capText.length() ? capText.toInt() : 0;
          motors.setDrive(l, r, p);
        }
      } else if (msg.startsWith("S,")) {
        if (!motors.isEmergencyStopped()) panServo.write(msg.substring(2).toInt());
      } else if (msg.startsWith("P,")) {
        String ip = msg.substring(2);
        ip.trim();
        if (startPing(ip)) roverLog("info", String("[PING] Pinging " + ip + " ...").c_str());
        else roverLog("warn", String("[PING] Cannot ping \"" + ip + "\" (busy or bad address)").c_str());
      } else if (msg == "CMD:ESTOP") {
        motors.emergencyStop("Dashboard E-STOP");
      } else if (msg == "CMD:RESET") {
        if (busIsOff()) roverLog("warn", "[E-STOP] Reset refused: no motor battery or E-stop switch OFF");
        else motors.resetEmergencyStop();
      } else if (msg == "PING" || msg == "T") {
        sendText("PONG");
      }
    } else if (opcode == 0x08) {
      _client.stop();
      _handshakeDone = false;
    }
  }

  void sendText(const char* text) {
    if (!bool(_client) || (_client.connected() == 0) || !_handshakeDone || !text) return;
    size_t len = strlen(text);
    uint8_t header[4];
    size_t hLen = 2;
    header[0] = 0x81;
    if (len < 126) { header[1] = (uint8_t)len; }
    else { header[1] = 126; header[2] = (uint8_t)(len >> 8); header[3] = (uint8_t)(len & 0xFF); hLen = 4; }
    _client.write(header, hLen);
    _client.write((const uint8_t*)text, len);
    // No _client.flush(): on the ESP32 it discards received bytes. Called 25x
    // a second here, it cut incoming drive frames in half, the parser then
    // read garbage as a "close" frame and the laptop link kept dropping.
  }
};

RoverWebSocketServer wsServer(WS_PORT);
WebServer httpServer(HTTP_PORT);

void roverLog(const char* level, const char* msg) {
  Serial.println(msg);

  LogEntry& entry = logBuffer[logHead];
  entry.id = nextLogId++;
  strncpy(entry.level, level ? level : "info", sizeof(entry.level) - 1);
  entry.level[sizeof(entry.level) - 1] = '\0';
  strncpy(entry.text, msg ? msg : "", sizeof(entry.text) - 1);
  entry.text[sizeof(entry.text) - 1] = '\0';
  logHead = (logHead + 1) % MAX_LOG_LINES;

  if (wsServer.hasClient()) {
    wsServer.logToDashboard(level, msg);
  }
}

unsigned long lastTelemTime = 0;
unsigned long lastStatusPrintTime = 0;
bool isApMode = false;

void handleRoot() {
  httpServer.sendHeader("Connection", "close");
  httpServer.send(200, "text/plain",
                  "Eco Rover ESP32-S3 is running.\n"
                  "The dashboard runs on the laptop: start laptop/server.py and open http://localhost:8080\n");
}

// Read-only. Driving over plain HTTP is switched off: it skipped the laptop's
// AUTO lock and E-stop checks, and anyone on the Wi-Fi could call it. The
// motors take commands only from the laptop's WebSocket on port 81.
void handleDrive() {
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"l\":%d,\"r\":%d,\"rl\":%d,\"rr\":%d,\"pwm\":%d,\"es\":%d,\"s1\":%d,\"s2\":%d,\"s3\":%d,\"s4\":%d,\"lid\":%u}",
           motors.getActualLeftPct(), motors.getActualRightPct(),
           motors.getActualLeftRaw(), motors.getActualRightRaw(),
           motors.getPwmCap(), motors.isEmergencyStopped() ? 1 : 0,
           sonars.getDistance(0), sonars.getDistance(1),
           sonars.getDistance(2), sonars.getDistance(3),
           nextLogId - 1);
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", buf);
}

void handleSonar() {
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"s1\":%d,\"s2\":%d,\"s3\":%d,\"s4\":%d}",
           sonars.getDistance(0), sonars.getDistance(1),
           sonars.getDistance(2), sonars.getDistance(3));
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", buf);
}

void handleLogs() {
  uint32_t afterId = httpServer.hasArg("after") ? httpServer.arg("after").toInt() : 0;
  String json = "{\"logs\":[";
  bool first = true;
  for (int i = 0; i < MAX_LOG_LINES; i++) {
    int idx = (logHead + i) % MAX_LOG_LINES;
    if (logBuffer[idx].id > afterId && logBuffer[idx].text[0] != '\0') {
      if (!first) json += ",";
      first = false;
      json += "{\"id\":" + String(logBuffer[idx].id) +
              ",\"lvl\":\"" + String(logBuffer[idx].level) +
              "\",\"msg\":\"" + String(logBuffer[idx].text) + "\"}";
    }
  }
  json += "]}";
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", json);
}

void handleEstop() {
  motors.emergencyStop("HTTP E-STOP");
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "text/plain", "OK");
}

// E-stop reset over plain HTTP is switched off: anyone on the Wi-Fi could
// re-arm the motors. Reset from the dashboard (WebSocket CMD:RESET) instead.
// Stopping stays open to everyone (GET /estop).
void handleReset() {
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(403, "text/plain", "Reset only from the dashboard (laptop)");
}

void handleStatus() {
  String json = "{\"uptime\":" + String(millis()) +
                ",\"left_pwm\":" + String(motors.getActualLeftPct()) +
                ",\"right_pwm\":" + String(motors.getActualRightPct()) +
                ",\"pwm_cap\":" + String(motors.getPwmCap()) +
                ",\"heading\":" + String(compass.heading()) +
                ",\"rssi\":" + String(WiFi.RSSI()) + "}";
  httpServer.sendHeader("Access-Control-Allow-Origin", "*");
  httpServer.send(200, "application/json", json);
}

// =============================================================================
// 7. MAIN ARDUINO SETUP & LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================================");
  Serial.println("  ECO-ECO ROVER · WIRELESS ESP32-S3 DIRECT DRIVE       ");
  Serial.println("  Left Motor  : GPIO 4 (RPWM), GPIO 5 (LPWM)           ");
  Serial.println("  Right Motor : GPIO 6 (RPWM), GPIO 7 (LPWM)           ");
  Serial.println("  Driver EN   : Tied to driver VCC 3.3V (Always Active) ");
  Serial.println("========================================================");

  // 1. Initialize Motors
  motors.begin();

  // 2. Initialize 4x Ultrasonic Sonars (HC-SR04)
  sonars.begin();

  // 2b. Camera pan servo on GPIO 8, centred
  panServo.begin();

  // 2c. Compass on I2C (GPIO 39 SDA, 40 SCL)
  compass.begin();

  roverLog("sys", "[BOOT] ESP32-S3 Firmware Initialized");
  roverLog("sys", "[MOTORS] BTS7960 Drivers Armed on GPIO 4, 5, 6, 7");
  roverLog("sys", "[SONAR] 4x HC-SR04 Active: S1(10,11) S2(12,13) S3(14,16) S4(17,18)");
  roverLog("sys", "[SERVO] MG996R pan servo on GPIO 8, centred at 90 deg");
  roverLog(compass.present() ? "sys" : "warn",
           compass.present() ? "[COMPASS] QMC5883P found on GPIO 39/40"
                             : "[COMPASS] Not found on GPIO 39/40 (heading = -1). Check wiring");
#if USE_BUS_SENSE
  roverLog("sys", "[E-STOP] Battery sense ON (GPIO 2): no battery or switch OFF = ESTOP");
#else
  roverLog("warn", "[E-STOP] Battery sense OFF (USE_BUS_SENSE false): bench mode only");
#endif

  // 3. Connect Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
#if USE_STATIC_IP
  WiFi.config(IPAddress(STATIC_IP), IPAddress(STATIC_GATEWAY), IPAddress(STATIC_SUBNET), IPAddress(STATIC_GATEWAY));
#endif
  Serial.printf("[WIFI] Connecting to network \"%s\"...\n", WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt) < WIFI_CONNECT_TIMEOUT) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    isApMode = false;
    roverLog("sys", String("[WIFI] Connected to SSID: \"" + String(WIFI_SSID) + "\"").c_str());
    roverLog("sys", String("[WIFI] Rover IP: " + WiFi.localIP().toString()).c_str());
  } else {
    isApMode = true;
    WiFi.mode(WIFI_AP);
    IPAddress apIP(AP_IP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    roverLog("warn", String("[WIFI] SoftAP Active: \"" + String(AP_SSID) + "\" @ " + WiFi.softAPIP().toString()).c_str());
  }

  // 3. Start mDNS (http://rover.local)
  if (MDNS.begin(MDNS_HOSTNAME)) {
    roverLog("sys", String("[mDNS] Responder Active: http://" + String(MDNS_HOSTNAME) + ".local").c_str());
    MDNS.addService("http", "tcp", HTTP_PORT);
    MDNS.addService("ws", "tcp", WS_PORT);
  }

  // 5. Start Web Server & WebSocket Server
  httpServer.on("/", HTTP_GET, handleRoot);
  httpServer.on("/drive", HTTP_GET, handleDrive);
  httpServer.on("/sonar", HTTP_GET, handleSonar);
  httpServer.on("/logs", HTTP_GET, handleLogs);
  httpServer.on("/estop", HTTP_GET, handleEstop);
  httpServer.on("/reset", HTTP_GET, handleReset);
  httpServer.on("/status", HTTP_GET, handleStatus);
  httpServer.begin();

  wsServer.begin();

  roverLog("sys", "[SERVER] HTTP REST API active on Port 80");
  roverLog("sys", "[READY] Rover is online & ready to drive!");
  Serial.println("========================================================\n");
}

void loop() {
  unsigned long now = millis();

  // Handle incoming HTTP browser requests (serves dashboard & REST motor drive)
  httpServer.handleClient();

  // Handle high-speed WebSocket motor control packets (25Hz)
  wsServer.update();

  // Update Motor PWMs (100 Hz / every 10ms with slew-rate limiter & deadtime)
  motors.update();

  // Update Non-blocking Ultrasonic Sensors (round-robin sweep across 4 sensors)
  sonars.update();

  // Compass heading (every 50 ms)
  compass.update();

  // Motor bus voltage: if the E-stop switch cuts motor power, latch the E-stop
  // so the rover does not start again by itself when the switch goes back ON.
  readBusVoltage();
  if (busIsOff() && !motors.isEmergencyStopped()) {
    motors.emergencyStop("No motor battery or E-stop switch OFF (motor bus < 6 V)");
  }

  // Report a finished QR ping to the laptop
  if (pingDone) {
    pingDone = false;
    char pBuf[64];
    snprintf(pBuf, sizeof(pBuf), "G,%s,%d,%u", pingTarget, pingReplies > 0 ? 1 : 0, (unsigned)pingRttMs);
    if (wsServer.hasClient()) wsServer.sendLine(pBuf);
    snprintf(pBuf, sizeof(pBuf), "[PING] %s: %u/4 replies, %u ms", pingTarget, (unsigned)pingReplies, (unsigned)pingRttMs);
    roverLog(pingReplies > 0 ? "sys" : "warn", pBuf);
  }

  // Stream drive state changes to Dashboard Terminal & Serial Monitor
  static int lastLoggedLeft = 0;
  static int lastLoggedRight = 0;
  int curL = motors.getActualLeftPct();
  int curR = motors.getActualRightPct();
  if (abs(curL - lastLoggedLeft) >= 20 || abs(curR - lastLoggedRight) >= 20 || 
      (curL == 0 && curR == 0 && (lastLoggedLeft != 0 || lastLoggedRight != 0))) {
    lastLoggedLeft = curL;
    lastLoggedRight = curR;
    if (curL == 0 && curR == 0) {
      roverLog("info", "[DRIVE] Motors Idle / Stopped (0%)");
    } else {
      char dBuf[80];
      const char* act = (curL > 0 && curR > 0) ? "FWD" : 
                        (curL < 0 && curR < 0) ? "REV" : 
                        (curL > curR) ? "RIGHT" : "LEFT";
      snprintf(dBuf, sizeof(dBuf), "[DRIVE] %s (L:%+d%% R:%+d%% Cap:%d%%)", act, curL, curR, motors.getPwmCap());
      roverLog("info", dBuf);
    }
  }

  // Send real-time telemetry to the dashboard via WebSocket if connected
  if (now - lastTelemTime >= 40) {
    lastTelemTime = now;
    if (wsServer.hasClient()) {
      wsServer.sendTelemetry(
        curL,
        curR,
        motors.getActualLeftRaw(),
        motors.getActualRightRaw(),
        sonars.getDistance(0),
        sonars.getDistance(1),
        sonars.getDistance(2),
        sonars.getDistance(3)
      );
    }
  }

  // Stream periodic Sonar telemetry to Dashboard Terminal & Serial Monitor (every 3s)
  static unsigned long lastSonarPrintTime = 0;
  if (now - lastSonarPrintTime >= 3000) {
    lastSonarPrintTime = now;
    int s1 = sonars.getDistance(0);
    int s2 = sonars.getDistance(1);
    int s3 = sonars.getDistance(2);
    int s4 = sonars.getDistance(3);
    char sBuf[128];
    char v1[12], v2[12], v3[12], v4[12];
    if (s1 > 0) snprintf(v1, sizeof(v1), "%dcm", s1); else snprintf(v1, sizeof(v1), "CLEAR");
    if (s2 > 0) snprintf(v2, sizeof(v2), "%dcm", s2); else snprintf(v2, sizeof(v2), "CLEAR");
    if (s3 > 0) snprintf(v3, sizeof(v3), "%dcm", s3); else snprintf(v3, sizeof(v3), "CLEAR");
    if (s4 > 0) snprintf(v4, sizeof(v4), "%dcm", s4); else snprintf(v4, sizeof(v4), "CLEAR");
    snprintf(sBuf, sizeof(sBuf), "[SONAR] S1:%s | S2:%s | S3:%s | S4:%s", v1, v2, v3, v4);
    roverLog("info", sBuf);
  }

  // Stream system monitoring to dashboard terminal & Serial Monitor (every 5s)
  if (now - lastStatusPrintTime >= 5000) {
    lastStatusPrintTime = now;
    char monBuf[128];
    snprintf(monBuf, sizeof(monBuf), "[MONITOR] Heap: %u KB | WiFi: %d dBm | Motors: %+d%% / %+d%%",
             ESP.getFreeHeap() / 1024, WiFi.RSSI(), curL, curR);
    roverLog("sys", monBuf);
  }
}
