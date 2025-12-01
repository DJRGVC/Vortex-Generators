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
 * Mode Toggle Switch (Pin 4):
 *   One terminal -> Arduino Pin 4
 *   Other terminal -> Arduino GND
 *   (Used to confirm/save calibration)
 *
 * Deployment Toggle Switch (Pin 10):
 *   One terminal -> Arduino Pin 10
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
 *   Pin 4  - Mode Toggle Switch (Calibration save confirmation)
 *   Pin 5  - Limit Switch
 *   Pin 6  - Motor AIN1 (PWM)
 *   Pin 7  - Encoder A (interrupt)
 *   Pin 8  - Encoder B
 *   Pin 9  - Motor AIN2 (PWM)
 *   Pin 10 - Deployment Toggle Switch (Manual positioning)
 *   Pin A0 - I2C SCL (IMU)
 *   Pin A1 - I2C SDA (IMU)
 * ============================================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include "../IMU_Sensor.h"
#include "../Pressure_Sensor.h"
#include "../Switch.h"

// Motor driver pins
#define MOTOR_AIN1 6
#define MOTOR_AIN2 9

// Encoder pins
#define ENCODER_A 7
#define ENCODER_B 8

// Switch pins
#define MODE_SWITCH_PIN 4        // Save confirmation
#define DEPLOY_SWITCH_PIN 10     // Manual positioning control
#define LIMIT_SWITCH_PIN 5

// EEPROM addresses (MUST match auto_control.ino)
#define EEPROM_MAGIC_ADDR 0      // Magic number (4 bytes)
#define EEPROM_MIN_ADDR 4        // Min position (4 bytes)
#define EEPROM_MAX_ADDR 8        // Max position (4 bytes)
#define EEPROM_IMU_OFFSET_ADDR 12 // IMU pitch offset at 0° angle of attack (4 bytes)
#define EEPROM_PRESSURE_BASELINE_ADDR 16 // Baseline pressure for airspeed calc (4 bytes)
#define EEPROM_MAGIC_NUMBER 0xCAFEBABE

// Motor control constants
#define CALIBRATION_SPEED 150
#define MANUAL_SPEED 100

// Encoder constants
#define ENCODER_CPR 12
#define GEAR_RATIO 248.98
#define COUNTS_PER_OUTPUT_REV (ENCODER_CPR * 4 * GEAR_RATIO)
#define COUNTS_PER_DEGREE (COUNTS_PER_OUTPUT_REV / 360.0)

// Position tracking
volatile long encoderCount = 0;
long calibrationMin = 0;
long calibrationMax = 0;
float imuPitchOffset = 0.0;  // IMU pitch at 0° angle of attack
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

  Serial.println("========================================");
  Serial.println("  CALIBRATION-ONLY MODE");
  Serial.println("========================================");
  Serial.println("");
  Serial.println("This sketch calibrates motor limits and");
  Serial.println("saves them to EEPROM for use by other sketches.");
  Serial.println("");

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
    Serial.println("ERROR: IMU initialization failed!");
    Serial.println("Check IMU connections:");
    Serial.println("  SDA -> Pin A1");
    Serial.println("  SCL -> Pin A0");
    Serial.println("  VIN -> 5V, GND -> GND");
    while (1);
  }

  // Initialize Pressure Sensor
  if (!pressureSensor.begin()) {
    Serial.println("ERROR: Pressure sensor initialization failed!");
    Serial.println("Check pressure sensor connections:");
    Serial.println("  SDA -> Pin A1");
    Serial.println("  SCL -> Pin A0");
    Serial.println("  VIN -> 5V, GND -> GND");
    while (1);
  }

  Serial.println("Hardware initialized!");
  Serial.println("");

  // Check if calibration already exists
  if (loadCalibration()) {
    Serial.println("WARNING: Existing calibration found in EEPROM:");
    Serial.print("  MIN: ");
    Serial.println(calibrationMin);
    Serial.print("  MAX: ");
    Serial.println(calibrationMax);
    Serial.print("  Range: ");
    Serial.print((float)calibrationMax / COUNTS_PER_DEGREE, 1);
    Serial.println(" degrees");
    Serial.print("  IMU Offset: ");
    Serial.print(imuPitchOffset, 2);
    Serial.println(" degrees");
    Serial.print("  Pressure Baseline: ");
    Serial.print(pressureBaseline, 2);
    Serial.println(" hPa");
    Serial.println("");
    Serial.println("This will be OVERWRITTEN by new calibration.");
    Serial.println("");
  } else {
    Serial.println("No existing calibration found.");
    Serial.println("");
  }

  delay(2000);

  // Run calibration
  runManualCalibration();
}

void loop() {
  // After calibration is complete, just display the results
  if (isCalibrated) {
    Serial.println("========================================");
    Serial.println("  CALIBRATION COMPLETE & SAVED");
    Serial.println("========================================");
    Serial.print("MIN Position: ");
    Serial.print(calibrationMin);
    Serial.println(" counts");
    Serial.print("MAX Position: ");
    Serial.print(calibrationMax);
    Serial.println(" counts");
    Serial.print("Range: ");
    Serial.print((float)calibrationMax / COUNTS_PER_DEGREE, 1);
    Serial.println(" degrees");
    Serial.print("IMU Offset: ");
    Serial.print(imuPitchOffset, 2);
    Serial.println("° (0° angle of attack)");
    Serial.print("Pressure Baseline: ");
    Serial.print(pressureBaseline, 2);
    Serial.println(" hPa (no airflow)");
    Serial.println("");
    Serial.println("You can now upload auto_control.ino");
    Serial.println("It will automatically load this calibration.");
    Serial.println("");
    Serial.println("Or reset this Arduino to recalibrate.");
    Serial.println("========================================");
    Serial.println("");

    // Infinite loop - calibration is done
    while(1) {
      delay(10000);
    }
  } else {
    Serial.println("ERROR: Calibration failed!");
    delay(1000);
  }
}

/*
 * Manual calibration routine
 */
void runManualCalibration() {
  Serial.println("=== STARTING MANUAL CALIBRATION ===");
  Serial.println("");

  // Step 1: Find MIN position at limit switch
  Serial.println("Step 1: Finding MIN position...");
  Serial.println("Moving clockwise to limit switch...");
  moveClockwise(CALIBRATION_SPEED);

  while (true) {
    limitSwitch.readState();
    if (limitSwitch.isOn()) {
      stopMotor();
      Serial.println("Limit switch hit!");
      break;
    }
    delay(10);
  }

  delay(500);

  // Step 2: Set MIN to 0
  encoderCount = 0;
  calibrationMin = 0;
  Serial.println("MIN position set to 0");
  Serial.println("");

  // Step 2.5: Read IMU pitch at this position (0° angle of attack)
  Serial.println("Reading IMU pitch at 0° angle of attack...");
  delay(500);  // Let motor settle

  // Take multiple readings and average
  float pitchSum = 0;
  int numReadings = 10;
  for (int i = 0; i < numReadings; i++) {
    imuSensor.readData();
    pitchSum += imuSensor.getPitch();
    delay(100);
  }
  imuPitchOffset = pitchSum / numReadings;

  Serial.print("IMU Offset calibrated: ");
  Serial.print(imuPitchOffset, 2);
  Serial.println("°");
  Serial.println("(This pitch value = 0° angle of attack)");
  Serial.println("");

  // Step 2.6: Read baseline pressure (no airflow)
  Serial.println("Reading baseline pressure (no airflow)...");
  Serial.println("IMPORTANT: Ensure NO airflow on pressure sensor!");
  delay(1000);

  // Take multiple readings and average
  float pressureSum = 0;
  for (int i = 0; i < numReadings; i++) {
    pressureSensor.readData();
    pressureSum += pressureSensor.getPressure();
    delay(100);
  }
  pressureBaseline = pressureSum / numReadings;

  Serial.print("Baseline Pressure: ");
  Serial.print(pressureBaseline, 2);
  Serial.println(" hPa");
  Serial.println("(This will be used to calculate airspeed)");
  Serial.println("");

  // Step 3: Manual positioning for MAX
  Serial.println("Step 3: Set MAX position manually");
  Serial.println("========================================");
  Serial.println("Controls:");
  Serial.println("  Pin 10 ON  = Move FORWARD (extend)");
  Serial.println("  Pin 10 OFF = Move BACKWARD (retract)");
  Serial.println("");
  Serial.println("To SAVE position:");
  Serial.println("  1. Toggle Pin 4 OFF");
  Serial.println("  2. Toggle Pin 4 ON");
  Serial.println("========================================");
  Serial.println("");

  bool lastDeployState = false;
  bool lastModeState = false;

  // Read initial states
  modeSwitch.readState();
  deploySwitch.readState();
  lastModeState = modeSwitch.isOn();
  lastDeployState = deploySwitch.isOn();

  unsigned long lastPrint = 0;

  while (true) {
    deploySwitch.readState();
    modeSwitch.readState();

    bool currentDeployState = deploySwitch.isOn();
    bool currentModeState = modeSwitch.isOn();

    // Check for save trigger (mode switch OFF -> ON)
    if (!lastModeState && currentModeState) {
      stopMotor();
      Serial.println("");
      Serial.println("SAVE TRIGGERED!");
      break;
    }

    // Control motor based on deploy switch
    if (currentDeployState != lastDeployState) {
      if (currentDeployState) {
        Serial.println(">> Moving FORWARD");
      } else {
        Serial.println(">> Moving BACKWARD");
      }
    }

    if (currentDeployState) {
      moveCounterClockwise(MANUAL_SPEED);
    } else {
      moveClockwise(MANUAL_SPEED);
    }

    // Print position periodically
    if (millis() - lastPrint > 500) {
      Serial.print("Current position: ");
      Serial.print(encoderCount);
      Serial.print(" (");
      Serial.print((float)encoderCount / COUNTS_PER_DEGREE, 1);
      Serial.println(" degrees)");
      lastPrint = millis();
    }

    lastDeployState = currentDeployState;
    lastModeState = currentModeState;
    delay(10);
  }

  // Step 4: Save calibration
  calibrationMax = encoderCount;

  Serial.println("");
  Serial.println("=== CALIBRATION VALUES ===");
  Serial.print("MIN: ");
  Serial.print(calibrationMin);
  Serial.println(" counts");
  Serial.print("MAX: ");
  Serial.print(calibrationMax);
  Serial.print(" counts (");
  Serial.print((float)calibrationMax / COUNTS_PER_DEGREE, 1);
  Serial.println(" degrees)");
  Serial.print("IMU Offset: ");
  Serial.print(imuPitchOffset, 2);
  Serial.println("° (pitch at 0° angle of attack)");
  Serial.print("Pressure Baseline: ");
  Serial.print(pressureBaseline, 2);
  Serial.println(" hPa (no airflow)");
  Serial.println("");

  // Save to EEPROM
  saveCalibration();
  Serial.println("Saved to EEPROM!");
  Serial.println("");

  // Verify by reading back
  if (loadCalibration()) {
    Serial.println("Verification: Successfully read back from EEPROM");
    Serial.print("  MIN: ");
    Serial.println(calibrationMin);
    Serial.print("  MAX: ");
    Serial.println(calibrationMax);
    Serial.print("  IMU Offset: ");
    Serial.println(imuPitchOffset, 2);
    Serial.print("  Pressure Baseline: ");
    Serial.println(pressureBaseline, 2);
  } else {
    Serial.println("ERROR: Failed to verify EEPROM write!");
  }

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
