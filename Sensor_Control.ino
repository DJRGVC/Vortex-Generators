/*
 * Multi-Sensor Control for Arduino Leonardo
 *
 * This sketch reads data from:
 * - BNO-055 IMU (orientation, gyroscope, accelerometer)
 * - MPRLS Pressure Sensor
 * - Toggle Switch
 * - Limit Switch (SS-5GL)
 *
 * Both sensors share the same I2C bus:
 * SCL -> Arduino A0 (SCL)
 * SDA -> Arduino A1 (SDA)
 *
 * Toggle Switch:
 * One terminal -> Digital Pin 4
 * Other terminal -> GND
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
#define TOGGLE_SWITCH_PIN 4
#define LIMIT_SWITCH_PIN 5

// Create sensor objects
IMU_Sensor imuSensor;
Pressure_Sensor pressureSensor;
Switch toggleSwitch(TOGGLE_SWITCH_PIN);
Switch limitSwitch(LIMIT_SWITCH_PIN);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for serial port to connect (Leonardo specific)

  Serial.println("Multi-Sensor System Starting...");
  Serial.println("");

  // Initialize I2C communication (shared by both sensors)
  Wire.begin();

  // Scan for I2C devices
  Serial.println("Scanning I2C bus...");
  scanI2C();
  Serial.println("");

  // Initialize IMU
  if (!imuSensor.begin()) {
    Serial.println("IMU initialization failed! Check connections.");
    while (1);
  }

  // Initialize Pressure Sensor
  if (!pressureSensor.begin()) {
    Serial.println("Pressure sensor initialization failed! Check connections.");
    while (1);
  }

  // Initialize Toggle Switch
  toggleSwitch.begin();

  // Initialize Limit Switch
  limitSwitch.begin();

  Serial.println("\nAll sensors initialized successfully!");
  Serial.println("Starting sensor readings...\n");
  delay(1000);
}

void loop() {
  // Read data from all sensors and switches
  imuSensor.readData();
  pressureSensor.readData();
  toggleSwitch.readState();
  limitSwitch.readState();

  // Print header
  Serial.println("=== Sensor Reading ===");

  // Print IMU data
  imuSensor.printData();

  // Print pressure data
  pressureSensor.printData();

  // Print toggle switch state
  Serial.print("Toggle ");
  toggleSwitch.printState();

  // Print limit switch state
  Serial.print("Limit  ");
  limitSwitch.printState();

  Serial.println();

  delay(SAMPLE_RATE_MS);
}

void scanI2C() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for I2C devices...");

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("Check your wiring:");
    Serial.println("  - SDA should be on pin A1");
    Serial.println("  - SCL should be on pin A0");
    Serial.println("  - VIN to 5V or 3.3V");
    Serial.println("  - GND to GND");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
}
