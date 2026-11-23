// Compass_Test — HMC5883L (or QMC5883L clone) bench test.
// Works on the ESP32 DevKit now and the ESP32-S3 later; the pins switch
// automatically by board.
//
// Wiring: VCC -> 3V3, GND -> GND, SDA/SCL -> pins below. Leave DRDY unconnected.
//   ESP32 DevKit : SDA 21, SCL 22
//   ESP32-S3     : SDA 8,  SCL 9   (spare in config_pins.h; NOT 21, that is E-stop sense)
//
// Serial Monitor 115200. Turn the sensor slowly through a full flat circle
// to build the hard-iron calibration. Send 'r' to reset the calibration.

#include <Wire.h>
#include <math.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
const int SDA_PIN = 8;
const int SCL_PIN = 9;
const char *BOARD = "ESP32-S3";
#else
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const char *BOARD = "ESP32 DevKit";
#endif

// Magnetic declination for your location, in degrees (east positive).
// Look it up at https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
const float DECLINATION_DEG = 0.0;

const uint8_t HMC_ADDR = 0x1E;
const uint8_t QMC_ADDR = 0x0D;
const uint8_t QMCP_ADDR = 0x2C;  // QMC5883P, newest GY-271 clone chip

enum Chip { NONE, HMC5883L, QMC5883L, QMC5883P };
Chip chip = NONE;

int16_t minX = 32767, maxX = -32768, minY = 32767, maxY = -32768;

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, count) != count) return false;
  for (uint8_t i = 0; i < count; i++) data[i] = Wire.read();
  return true;
}

bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void scanBus() {
  Serial.print("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (present(a)) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  Serial.println(found ? "" : " nothing found - check VCC, GND, SDA, SCL");
}

bool initHmc() {
  uint8_t id[3];
  if (!readRegs(HMC_ADDR, 0x0A, 3, id)) return false;
  if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
    Serial.printf("0x1E answered but ID is %02X %02X %02X, not H43\n", id[0], id[1], id[2]);
    return false;
  }
  writeReg(HMC_ADDR, 0x00, 0x70);  // 8-sample average, 15 Hz
  writeReg(HMC_ADDR, 0x01, 0x20);  // gain 1090 LSB/G (+-1.3 G)
  writeReg(HMC_ADDR, 0x02, 0x00);  // continuous
  return true;
}

bool initQmc() {
  if (!present(QMC_ADDR)) return false;
  writeReg(QMC_ADDR, 0x0B, 0x01);  // SET/RESET period
  writeReg(QMC_ADDR, 0x09, 0x1D);  // continuous, 200 Hz, 8 G, OSR 512
  return true;
}

bool initQmcP() {
  uint8_t id;
  if (!readRegs(QMCP_ADDR, 0x00, 1, &id) || id != 0x80) return false;
  writeReg(QMCP_ADDR, 0x29, 0x06);  // axis sign setup (datasheet example)
  writeReg(QMCP_ADDR, 0x0B, 0x08);  // SET/RESET on, +-8 G
  writeReg(QMCP_ADDR, 0x0A, 0xCD);  // normal mode, 200 Hz, OSR 8/8
  return true;
}

// Returns false on a bus error or a saturated axis.
bool readMag(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t d[6];
  if (chip == HMC5883L) {
    if (!readRegs(HMC_ADDR, 0x03, 6, d)) return false;
    x = (int16_t)((d[0] << 8) | d[1]);  // order is X, Z, Y, big-endian
    z = (int16_t)((d[2] << 8) | d[3]);
    y = (int16_t)((d[4] << 8) | d[5]);
    if (x == -4096 || y == -4096 || z == -4096) {
      Serial.println("Overflow: field too strong (magnet/motor nearby?)");
      return false;
    }
  } else {
    if (chip == QMC5883P) {
      if (!readRegs(QMCP_ADDR, 0x01, 6, d)) return false;
    } else if (!readRegs(QMC_ADDR, 0x00, 6, d)) {
      return false;
    }
    x = (int16_t)(d[0] | (d[1] << 8));  // order is X, Y, Z, little-endian
    y = (int16_t)(d[2] | (d[3] << 8));
    z = (int16_t)(d[4] | (d[5] << 8));
  }
  return true;
}

void resetCal() {
  minX = minY = 32767;
  maxX = maxY = -32768;
  Serial.println("Calibration reset - turn the sensor through a full circle");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== Compass_Test (%s, SDA %d, SCL %d) ===\n", BOARD, SDA_PIN, SCL_PIN);
  Wire.begin(SDA_PIN, SCL_PIN, 100000);

  scanBus();
  if (initHmc()) {
    chip = HMC5883L;
    Serial.println("Found HMC5883L at 0x1E");
  } else if (initQmc()) {
    chip = QMC5883L;
    Serial.println("Found QMC5883L at 0x0D (clone chip - common on GY-271 boards)");
  } else if (initQmcP()) {
    chip = QMC5883P;
    Serial.println("Found QMC5883P at 0x2C (clone chip, ID 0x80)");
  } else {
    Serial.println("No compass found. Check wiring, then reset the board.");
    while (true) delay(1000);
  }
  delay(20);
}

void loop() {
  if (Serial.available() && Serial.read() == 'r') resetCal();

  int16_t x, y, z;
  if (!readMag(x, y, z)) {
    Serial.println("Could not read sensor");
    delay(500);
    return;
  }

  if (x < minX) minX = x;
  if (x > maxX) maxX = x;
  if (y < minY) minY = y;
  if (y > maxY) maxY = y;

  float offX = (maxX + minX) / 2.0;
  float offY = (maxY + minY) / 2.0;
  float heading = atan2(y - offY, x - offX) * 180.0 / PI + DECLINATION_DEG;
  if (heading < 0) heading += 360.0;
  if (heading >= 360) heading -= 360.0;

  Serial.printf("X %6d  Y %6d  Z %6d | offset X %6.0f Y %6.0f | heading %5.1f deg\n",
                x, y, z, offX, offY, heading);
  delay(200);
}
