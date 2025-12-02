/*
 * Multi-Sensor Control for Arduino Leonardo
 *
 * This sketch reads data from:
 * - BNO-055 IMU (orientation, gyroscope, accelerometer)
 * - MPRLS Pressure Sensor
 * - Mode Toggle Switch (Pin 4) - Manual/Automatic mode selection
 * - Deployment Toggle Switch (Pin 10) - Deployment control
 * - Limit Switch (SS-5GL)
 *
 * Both sensors share the same I2C bus:
 * SCL -> Arduino A0 (SCL)
 * SDA -> Arduino A1 (SDA)
 *
 * Mode Toggle Switch (Pin 4):
 * One terminal -> Digital Pin 4
 * Other terminal -> GND
 * (OFF = Manual Mode, ON = Automatic Mode)
 *
 * Deployment Toggle Switch (Pin 10):
 * One terminal -> Digital Pin 10
 * Other terminal -> GND
 * (Used in Manual Mode: OFF = Retracted, ON = Extended)
 *
 * Limit Switch (SS-5GL):
 * COM terminal -> Digital Pin 5
 * NO terminal -> GND
 * NC terminal -> (not connected)
 */

#include <Wire.h>
#include "IMU_Sensor.h"
#include "Pressure_Sensor.h"
#include "Switch.h"

// Set the delay between fresh samples (milliseconds)
#define SAMPLE_RATE_MS (100)

// Define switch pins (avoid pins 2 & 3 - they're used for I2C!)
#define MODE_SWITCH_PIN 4        // Manual/Automatic mode toggle
#define DEPLOY_SWITCH_PIN 10     // Deployment control toggle
#define LIMIT_SWITCH_PIN 5

// Create sensor objects
IMU_Sensor imuSensor;
Pressure_Sensor pressureSensor;
Switch modeSwitch(MODE_SWITCH_PIN);
Switch deploySwitch(DEPLOY_SWITCH_PIN);
Switch limitSwitch(LIMIT_SWITCH_PIN);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("=== Sensor System ==="));
  Serial.println(F("Leonardo: SDA=A1, SCL=A0"));

  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("I2C @ 400kHz"));

  // Scan for I2C devices
  scanI2C();

  // Initialize IMU
  if (!imuSensor.begin()) {
    Serial.println(F("IMU FAILED!"));
    while (1);
  }

  // Initialize Pressure Sensor
  if (!pressureSensor.begin()) {
    Serial.println(F("Pressure FAILED!"));
    while (1);
  }

  // Initialize switches
  modeSwitch.begin();
  deploySwitch.begin();
  limitSwitch.begin();

  Serial.println(F("\nAll systems GO!\n"));
  delay(1000);
}

void loop() {
  // Read data from all sensors and switches
  imuSensor.readData();
  pressureSensor.readData();
  modeSwitch.readState();
  deploySwitch.readState();
  limitSwitch.readState();

  // Get angle of attack (pitch) and acceleration in Z-axis (vertical)
  float angleOfAttack = imuSensor.getPitch();
  float accelZ = imuSensor.getAccelZ();

  // Print header
  Serial.println("=== Sensor Reading ===");

  // Print angle of attack
  Serial.print("Angle of Attack: ");
  Serial.print(angleOfAttack, 2);
  Serial.println("°");

  // Print acceleration in pitch axis (Z-axis - vertical)
  Serial.print("Accel (Z-axis): ");
  Serial.print(accelZ, 2);
  Serial.println(" m/s²");

  // Print pressure data
  pressureSensor.printData();

  // Print mode toggle switch state (Manual/Auto)
  Serial.print("Mode Switch (Pin 4):   ");
  modeSwitch.printState();

  // Print deployment toggle switch state (Retract/Extend)
  Serial.print("Deploy Switch (Pin 10): ");
  deploySwitch.printState();

  // Print limit switch state
  Serial.print("Limit Switch (Pin 5):   ");
  limitSwitch.printState();

  Serial.println();

  delay(SAMPLE_RATE_MS);
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;

  Serial.println(F("I2C Scan..."));

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("  0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);

      if (address == 0x28 || address == 0x29) {
        Serial.print(F(" (BNO055)"));
      } else if (address == 0x18) {
        Serial.print(F(" (MPRLS)"));
      }
      Serial.println();
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println(F("NO DEVICES! Check wiring."));
  } else {
    Serial.print(F("Found "));
    Serial.print(nDevices);
    Serial.println(F(" device(s)"));
  }
}
