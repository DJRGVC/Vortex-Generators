/*
 * Calibration-Only Mode for Vortex Generator Motor Control
 *
 * This standalone sketch is used ONLY for calibrating the motor limits.
 * Calibration values are saved to EEPROM and will be automatically loaded
 * by auto_control.ino when you switch back to that sketch.
 *
 * HOW TO USE:
 * 1. Upload this sketch to your Arduino
 * 2. Follow the on-screen prompts to calibrate
 * 3. When done, calibration is saved to EEPROM
 * 4. Upload auto_control.ino - it will use the saved calibration
 *
 * CALIBRATION PROCESS:
 * 1. Motor moves clockwise to find limit switch (MIN position = 0)
 * 2. IMU pitch is read at this position (assumed 0° angle of attack)
 * 3. IMU offset saved to EEPROM for automatic mode compensation
 * 4. Use Pin 10 toggle to manually position motor to desired MAX position
 *    - Toggle ON = move forward (extend)
 *    - Toggle OFF = move backward (retract)
 * 5. Toggle Pin 4 OFF then ON to save MAX position
 * 6. Motor and IMU calibration saved to EEPROM and displayed
 *
 * ============================================================================
 * WIRING REQUIRED FOR CALIBRATION:
 * ============================================================================
 *
 * SENSORS (I2C):
 * -----------------------------------
 * BNO-055 IMU:
 *   VIN  -> Arduino 5V (or 3.3V)
 *   GND  -> Arduino GND
 *   SDA  -> Arduino A1
 *   SCL  -> Arduino A0
 *
 * SWITCHES:
 * -----------------------------------
 * Mode Toggle Switch (Pin 10):
 *   One terminal -> Arduino Pin 10
 *   Other terminal -> Arduino GND
 *   (Used to confirm/save calibration)
 *
 * Deployment Toggle Switch (Pin 4):
 *   One terminal -> Arduino Pin 4
 *   Other terminal -> Arduino GND
 *   (Used to position motor during calibration)
 *
 * Limit Switch (SS-5GL):
 *   COM terminal -> Arduino Pin 5
 *   NO terminal  -> Arduino GND
 *   NC terminal  -> (not connected)
 *
 * MOTOR & DRIVER (DRV8833):
 * -----------------------------------
 * DRV8833 Motor Driver:
 *   VCC  -> Arduino 5V
 *   GND  -> Arduino GND
 *   AIN1 -> Arduino Pin 6
 *   AIN2 -> Arduino Pin 9
 *   SLP  -> Arduino 5V (or VCC) [if present]
 *
 * Motor 6-Pin Connector (Pololu Micro Metal Gearmotor):
 *   Pin 1 (Red)    - Motor M1   -> DRV8833 AOUT1
 *   Pin 2 (Black)  - Motor M2   -> DRV8833 AOUT2
 *   Pin 3 (Blue)   - Encoder Vcc -> Arduino 5V
 *   Pin 4 (Yellow) - Encoder A  -> Arduino Pin 7
 *   Pin 5 (White)  - Encoder B  -> Arduino Pin 8
 *   Pin 6 (Green)  - Encoder GND -> Arduino GND
 *
 * NOTE: Pressure sensor is NOT required for calibration
 *       IMU is REQUIRED to calibrate 0° angle of attack reference
 *
 * ============================================================================
 * PIN USAGE SUMMARY:
 * ============================================================================
 *   Pin 4  - Deployment Toggle Switch (Manual positioning)
 *   Pin 5  - Limit Switch
 *   Pin 6  - Motor AIN1 (PWM)
 *   Pin 7  - Encoder A (interrupt)
 *   Pin 8  - Encoder B
 *   Pin 9  - Motor AIN2 (PWM)
 *   Pin 10 - Mode Toggle Switch (Calibration save confirmation)
 *   Pin A0 - I2C SCL (IMU)
 *   Pin A1 - I2C SDA (IMU)
 * ============================================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include "IMU_Sensor.h"
#include "Pressure_Sensor.h"
#include "Switch.h"

// Motor driver pins
#define MOTOR_AIN1 6
#define MOTOR_AIN2 9

// Encoder pins
#define ENCODER_A 7
#define ENCODER_B 8

// Switch pins
#define MODE_SWITCH_PIN 10       // Save confirmation
#define DEPLOY_SWITCH_PIN 4      // Manual positioning control
#define LIMIT_SWITCH_PIN 5

// EEPROM addresses (MUST match auto_control.ino)
#define EEPROM_MAGIC_ADDR 0      // Magic number (4 bytes)
#define EEPROM_MIN_ADDR 4        // Min position (4 bytes)
#define EEPROM_MAX_ADDR 8        // Max position (4 bytes)
#define EEPROM_IMU_OFFSET_ADDR 12 // IMU roll offset at 0° angle of attack (4 bytes)
#define EEPROM_PRESSURE_BASELINE_ADDR 16 // Baseline pressure for airspeed calc (4 bytes)
#define EEPROM_MAGIC_NUMBER 0xCAFEBABE

// Motor control constants
#define CALIBRATION_SPEED 45      // Slow speed for finding limit switch
#define MANUAL_SPEED 45           // Slow speed for manual positioning

// Encoder constants
#define ENCODER_CPR 12
#define GEAR_RATIO 248.98
#define COUNTS_PER_OUTPUT_REV (ENCODER_CPR * 4 * GEAR_RATIO)
#define COUNTS_PER_DEGREE (COUNTS_PER_OUTPUT_REV / 360.0)

// Position tracking
volatile long encoderCount = 0;
long calibrationMin = 0;
long calibrationMax = 0;
float imuPitchOffset = 0.0;  // IMU roll at 0° angle of attack
float pressureBaseline = 0.0;  // Baseline pressure (no airflow) in hPa
bool isCalibrated = false;

// Sensor objects
IMU_Sensor imuSensor;
Pressure_Sensor pressureSensor;

// Switch objects
Switch modeSwitch(MODE_SWITCH_PIN);
Switch deploySwitch(DEPLOY_SWITCH_PIN);
Switch limitSwitch(LIMIT_SWITCH_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("=== CALIBRATION MODE ==="));
  Serial.println(F("Starting in 3 sec..."));
  delay(3000);

  // Initialize motor pins
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  stopMotor();

  // Initialize encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // Initialize switches
  modeSwitch.begin();
  deploySwitch.begin();
  limitSwitch.begin();

  // Initialize I2C
  Wire.begin();

  // Initialize IMU
  if (!imuSensor.begin()) {
    Serial.println(F("IMU FAILED!"));
    while (1);
  }

  // Initialize Pressure Sensor
  if (!pressureSensor.begin()) {
    Serial.println(F("PRESSURE FAILED!"));
    while (1);
  }

  Serial.println(F("HW OK"));

  // Check if calibration already exists
  if (loadCalibration()) {
    Serial.println(F("Old cal found - will overwrite"));
  }

  delay(2000);

  // Run calibration
  runManualCalibration();
}

void loop() {
  if (isCalibrated) {
    Serial.println(F("=== DONE ==="));
    Serial.println(F("Upload auto_control.ino"));
    while(1) {
      delay(10000);
    }
  } else {
    Serial.println(F("FAIL!"));
    delay(1000);
  }
}

/*
 * Manual calibration routine
 */
void runManualCalibration() {
  Serial.println(F("=== CALIBRATION ==="));

  // Step 1: Find MIN position at limit switch
  Serial.println(F("Finding MIN..."));
  Serial.println(F("Moving CCW to limit..."));

  moveCounterClockwise(CALIBRATION_SPEED);

  unsigned long startTime = millis();
  unsigned long timeout = 15000;  // 15 second timeout
  unsigned long lastDebug = 0;

  while (true) {
    limitSwitch.readState();
    bool switchState = limitSwitch.isOn();

    // Debug output every 500ms
    if (millis() - lastDebug > 500) {
      Serial.print(F("Lim: "));
      Serial.println(switchState ? F("ON") : F("OFF"));
      lastDebug = millis();
    }

    if (switchState) {
      stopMotor();
      Serial.println(F("MIN found!"));
      break;
    }

    // Timeout check
    if (millis() - startTime > timeout) {
      stopMotor();
      Serial.println(F("Timeout - using current pos"));
      break;
    }

    delay(10);
  }

  delay(500);

  // Step 2: Set MIN to 0
  encoderCount = 0;
  calibrationMin = 0;
  Serial.println(F("MIN = 0"));

  // Step 2.5: Read IMU roll at this position (0° angle of attack)
  Serial.println(F("Reading IMU..."));
  delay(500);

  float pitchSum = 0;
  int numReadings = 5;
  for (int i = 0; i < numReadings; i++) {
    imuSensor.readData();
    pitchSum += imuSensor.getRoll();
    delay(100);
  }
  imuPitchOffset = pitchSum / numReadings;

  Serial.print(F("IMU: "));
  Serial.println(imuPitchOffset, 2);

  // Step 2.6: Read baseline pressure (no airflow)
  Serial.println(F("Reading pressure..."));
  delay(500);

  float pressureSum = 0;
  for (int i = 0; i < numReadings; i++) {
    pressureSensor.readData();
    pressureSum += pressureSensor.getPressure_hPa();
    delay(100);
  }
  pressureBaseline = pressureSum / numReadings;

  Serial.print(F("P: "));
  Serial.println(pressureBaseline, 2);

  // Step 3: Manual positioning for MAX
  Serial.println(F("Set MAX:"));
  Serial.println(F("Pin4 ON=CW, OFF=Stop"));
  Serial.println(F("Pin10 ON=Save"));

  bool lastDeployState = false;
  unsigned long lastPrint = 0;

  while (true) {
    deploySwitch.readState();
    modeSwitch.readState();

    bool currentDeployState = deploySwitch.isOn();
    bool currentModeState = modeSwitch.isOn();

    // Check if mode switch is ON to save
    if (currentModeState) {
      stopMotor();
      Serial.println(F("SAVING!"));
      break;
    }

    // Control motor based on deploy switch
    if (currentDeployState != lastDeployState) {
      if (currentDeployState) {
        Serial.println(F("CW"));
      } else {
        Serial.println(F("STOP"));
      }
    }

    // Deploy switch ON = move clockwise (extend), OFF = stop
    if (currentDeployState) {
      moveClockwise(MANUAL_SPEED);
    } else {
      stopMotor();
    }

    // Print position periodically
    if (millis() - lastPrint > 500) {
      Serial.print(F("Pos: "));
      Serial.println(encoderCount);
      lastPrint = millis();
    }

    lastDeployState = currentDeployState;
    delay(10);
  }

  // Step 4: Save calibration
  calibrationMax = encoderCount;

  Serial.print(F("MIN: "));
  Serial.println(calibrationMin);
  Serial.print(F("MAX: "));
  Serial.println(calibrationMax);

  saveCalibration();
  Serial.println(F("Saved!"));

  isCalibrated = true;
  Serial.println("");
  delay(2000);
}

/*
 * EEPROM Functions (MUST match auto_control.ino)
 */
void saveCalibration() {
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_NUMBER);
  EEPROM.put(EEPROM_MIN_ADDR, calibrationMin);
  EEPROM.put(EEPROM_MAX_ADDR, calibrationMax);
  EEPROM.put(EEPROM_IMU_OFFSET_ADDR, imuPitchOffset);
  EEPROM.put(EEPROM_PRESSURE_BASELINE_ADDR, pressureBaseline);
}

bool loadCalibration() {
  unsigned long magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic != EEPROM_MAGIC_NUMBER) {
    return false;
  }

  EEPROM.get(EEPROM_MIN_ADDR, calibrationMin);
  EEPROM.get(EEPROM_MAX_ADDR, calibrationMax);
  EEPROM.get(EEPROM_IMU_OFFSET_ADDR, imuPitchOffset);
  EEPROM.get(EEPROM_PRESSURE_BASELINE_ADDR, pressureBaseline);

  if (calibrationMax <= calibrationMin || calibrationMax > 10000) {
    return false;
  }

  return true;
}

/*
 * Motor control functions
 */
void moveClockwise(int speed) {
  analogWrite(MOTOR_AIN1, speed);
  analogWrite(MOTOR_AIN2, 0);
}

void moveCounterClockwise(int speed) {
  analogWrite(MOTOR_AIN1, 0);
  analogWrite(MOTOR_AIN2, speed);
}

void stopMotor() {
  analogWrite(MOTOR_AIN1, 0);
  analogWrite(MOTOR_AIN2, 0);
}

/*
 * Encoder interrupt service routine
 */
void encoderISR() {
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  if (a == b) {
    encoderCount++;  // Counterclockwise
  } else {
    encoderCount--;  // Clockwise
  }
}
