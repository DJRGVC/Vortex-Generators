/*
 * Read EEPROM Calibration Data
 *
 * This sketch reads and displays calibration data saved to EEPROM
 * by the calibration_only.ino sketch.
 */

#include <EEPROM.h>

// EEPROM addresses (must match calibration_only.ino)
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MIN_ADDR 4
#define EEPROM_MAX_ADDR 8
#define EEPROM_IMU_OFFSET_ADDR 12
#define EEPROM_PRESSURE_BASELINE_ADDR 16
#define EEPROM_MAGIC_NUMBER 0xCAFEBABE

// Encoder constants (for degree conversion)
#define ENCODER_CPR 12
#define GEAR_RATIO 248.98
#define COUNTS_PER_OUTPUT_REV (ENCODER_CPR * 4 * GEAR_RATIO)
#define COUNTS_PER_DEGREE (COUNTS_PER_OUTPUT_REV / 360.0)

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println(F("EEPROM Reader Ready"));
}

void loop() {
  Serial.println(F("=== EEPROM DATA ==="));

  // Read magic number
  unsigned long magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  Serial.print(F("Magic: 0x"));
  Serial.println(magic, HEX);

  if (magic != EEPROM_MAGIC_NUMBER) {
    Serial.println(F("NO VALID CAL!"));
    Serial.print(F("Expected: 0x"));
    Serial.println(EEPROM_MAGIC_NUMBER, HEX);
  } else {
    // Read calibration values
    long calibrationMin;
    long calibrationMax;
    float imuOffset;
    float pressureBaseline;

    EEPROM.get(EEPROM_MIN_ADDR, calibrationMin);
    EEPROM.get(EEPROM_MAX_ADDR, calibrationMax);
    EEPROM.get(EEPROM_IMU_OFFSET_ADDR, imuOffset);
    EEPROM.get(EEPROM_PRESSURE_BASELINE_ADDR, pressureBaseline);

    // Print values
    Serial.print(F("MIN: "));
    Serial.println(calibrationMin);

    Serial.print(F("MAX: "));
    Serial.println(calibrationMax);

    Serial.print(F("Range: "));
    Serial.print((float)(calibrationMax - calibrationMin) / COUNTS_PER_DEGREE, 1);
    Serial.println(F(" deg"));

    Serial.print(F("IMU Offset: "));
    Serial.println(imuOffset, 2);

    Serial.print(F("Pressure: "));
    Serial.println(pressureBaseline, 2);
  }

  Serial.println();
  delay(2000);  // Print every 2 seconds
}
