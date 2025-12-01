/*
 * Vortex Generator Control with Manual/Automatic Modes
 *
 * This sketch controls vortex generators using a Pololu Micro Metal Gearmotor
 * with encoder and DRV8833 motor driver.
 *
 * MODES:
 * - Manual Mode (Pin 4 toggle OFF): Pin 10 toggle controls deployment
 *   - Pin 10 OFF: Fully retracted (0 degrees)
 *   - Pin 10 ON: Fully extended (65 degrees)
 * - Automatic Mode (Pin 4 toggle ON): IMU/Pressure sensor controls deployment
 *   - Motor position based on IMU pitch and pressure readings
 *
 * CALIBRATION MODE:
 * - To enter: Hold BOTH toggle switches (Pin 4 and Pin 10) ON during startup
 * - Calibration process:
 *   1. Move clockwise until limit switch is hit (saves MIN position = 0)
 *   2. User manually positions motor to desired MAX position using Pin 10 toggle
 *   3. Press Pin 4 toggle to save MAX position to EEPROM
 * - Calibration values persist across power cycles
 * - If no calibration exists, system will prompt for calibration
 *
 * Normal startup (toggles not both ON):
 * - Load saved calibration from EEPROM
 * - Enter selected mode (manual or automatic)
 *
 * ============================================================================
 * COMPLETE WIRING GUIDE - ALL CONNECTIONS
 * ============================================================================
 *
 * SENSORS (I2C - all share same bus):
 * -----------------------------------
 * BNO-055 IMU:
 *   VIN  -> Arduino 5V (or 3.3V)
 *   GND  -> Arduino GND
 *   SDA  -> Arduino A1
 *   SCL  -> Arduino A0
 *
 * MPRLS Pressure Sensor:
 *   VIN  -> Arduino 5V (or 3.3V)
 *   GND  -> Arduino GND
 *   SDA  -> Arduino A1 (shared with IMU)
 *   SCL  -> Arduino A0 (shared with IMU)
 *
 * SWITCHES:
 * -----------------------------------
 * Mode Toggle Switch (Pin 4):
 *   One terminal -> Arduino Pin 4
 *   Other terminal -> Arduino GND
 *   (OFF = Manual Mode, ON = Automatic Mode)
 *
 * Deployment Toggle Switch (Pin 10):
 *   One terminal -> Arduino Pin 10
 *   Other terminal -> Arduino GND
 *   (Used in Manual Mode: OFF = Retracted, ON = Extended)
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
 *   SLP  -> Arduino 5V (or VCC) [if present - keeps driver awake]
 *
 * Motor 6-Pin Connector (Pololu Micro Metal Gearmotor):
 *   Pin 1 (Red)    - Motor M1   -> DRV8833 AOUT1
 *   Pin 2 (Black)  - Motor M2   -> DRV8833 AOUT2
 *   Pin 3 (Blue)   - Encoder Vcc -> Arduino 5V
 *   Pin 4 (Yellow) - Encoder A  -> Arduino Pin 7
 *   Pin 5 (White)  - Encoder B  -> Arduino Pin 8
 *   Pin 6 (Green)  - Encoder GND -> Arduino GND
 *
 * POWER:
 * -----------------------------------
 * Arduino Leonardo:
 *   USB power (for testing) OR 7-12V via barrel jack
 *
 * IMPORTANT NOTES:
 *   - Motor rated for 6V but running at 5V (lower speed/torque)
 *   - All GND connections must be common
 *   - For production use, consider separate 6V supply for motor
 *
 * ============================================================================
 * PIN USAGE SUMMARY:
 * ============================================================================
 *   Pin 4  - Mode Toggle Switch (Manual/Automatic)
 *   Pin 5  - Limit Switch
 *   Pin 6  - Motor AIN1 (PWM)
 *   Pin 7  - Encoder A (interrupt)
 *   Pin 8  - Encoder B
 *   Pin 9  - Motor AIN2 (PWM)
 *   Pin 10 - Deployment Toggle Switch (Manual mode control)
 *   Pin A0 - I2C SCL (IMU & Pressure)
 *   Pin A1 - I2C SDA (IMU & Pressure)
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

// Encoder pins (Pin 7 has interrupt capability on Leonardo)
#define ENCODER_A 7
#define ENCODER_B 8

// Switch pins
#define MODE_SWITCH_PIN 4        // Manual/Automatic mode selection
#define DEPLOY_SWITCH_PIN 10     // Deployment control (manual mode)
#define LIMIT_SWITCH_PIN 5

// EEPROM addresses for calibration storage
#define EEPROM_MAGIC_ADDR 0      // Magic number to verify valid calibration
#define EEPROM_MIN_ADDR 4        // Min position (4 bytes for long)
#define EEPROM_MAX_ADDR 8        // Max position (4 bytes for long)
#define EEPROM_IMU_OFFSET_ADDR 12 // IMU pitch offset at 0° angle of attack (4 bytes for float)
#define EEPROM_MAGIC_NUMBER 0xCAFEBABE  // Magic number to verify EEPROM data

// Motor control constants
#define CALIBRATION_SPEED 150     // PWM speed during calibration (0-255)
#define CONTROL_SPEED 200         // PWM speed during normal operation
#define CALIBRATION_DEGREES 65.0  // Degrees to rotate back from limit

// Encoder constants
#define ENCODER_CPR 12            // Counts per revolution on motor shaft
#define GEAR_RATIO 248.98         // Gearbox ratio
#define COUNTS_PER_OUTPUT_REV (ENCODER_CPR * 4 * GEAR_RATIO)  // 11,951 counts
#define COUNTS_PER_DEGREE (COUNTS_PER_OUTPUT_REV / 360.0)     // ~33.2 counts per degree

// Position tracking
volatile long encoderCount = 0;
long calibrationMin = 0;  // Minimum position (at limit switch)
long calibrationMax = 0;  // Maximum position (user-defined)
float imuPitchOffset = 0.0;  // IMU pitch at 0° angle of attack
bool isCalibrated = false;

// IMU pitch mapping for automatic mode
#define IMU_PITCH_MIN -30.0   // IMU pitch that maps to motor position 0°
#define IMU_PITCH_MAX 30.0    // IMU pitch that maps to motor position 65°

// Pressure mapping for automatic mode (adjust based on your application)
#define PRESSURE_MIN 1000.0   // Pressure (hPa) that maps to motor position 0°
#define PRESSURE_MAX 1020.0   // Pressure (hPa) that maps to motor position 65°

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

  Serial.println("Vortex Generator Control System Starting...");
  Serial.println("");

  // Initialize motor control pins
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  stopMotor();

  // Initialize encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);

  // Initialize I2C
  Wire.begin();

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

  // Initialize switches
  modeSwitch.begin();
  deploySwitch.begin();
  limitSwitch.begin();

  Serial.println("All systems initialized!");
  Serial.println("");
  delay(1000);

  // Check if both toggles are ON for calibration mode
  modeSwitch.readState();
  deploySwitch.readState();
  bool enterCalibrationMode = modeSwitch.isOn() && deploySwitch.isOn();

  if (enterCalibrationMode) {
    Serial.println("BOTH TOGGLES ON - ENTERING CALIBRATION MODE");
    Serial.println("");
    runManualCalibration();
  } else {
    // Try to load saved calibration
    if (loadCalibration()) {
      Serial.println("Loaded saved calibration from EEPROM");
      Serial.print("Min position: ");
      Serial.println(calibrationMin);
      Serial.print("Max position: ");
      Serial.println(calibrationMax);
      Serial.print("IMU Offset: ");
      Serial.print(imuPitchOffset, 2);
      Serial.println("° (0° angle of attack)");
      Serial.println("");
      isCalibrated = true;
    } else {
      Serial.println("WARNING: No saved calibration found!");
      Serial.println("Please restart with both toggles ON to calibrate.");
      Serial.println("");
      while(1);  // Halt - require calibration
    }
  }

  delay(1000);
}

void loop() {
  if (!isCalibrated) {
    Serial.println("ERROR: Motor not calibrated!");
    delay(1000);
    return;
  }

  // Read mode switch to determine control mode
  modeSwitch.readState();
  bool isAutomaticMode = modeSwitch.isOn();

  long targetPosition;

  if (isAutomaticMode) {
    // AUTOMATIC MODE: Use IMU and pressure sensor
    imuSensor.readData();
    pressureSensor.readData();

    float pitchRaw = imuSensor.getPitch();
    float pitch = pitchRaw - imuPitchOffset;  // Apply calibration offset
    float pressure = pressureSensor.getPressure();

    // Map sensor readings to motor position
    targetPosition = calculateAutomaticPosition(pitch, pressure);

    // Print automatic mode status
    Serial.print("AUTO | Pitch: ");
    Serial.print(pitch, 1);
    Serial.print("° (raw: ");
    Serial.print(pitchRaw, 1);
    Serial.print("°) | Pressure: ");
    Serial.print(pressure, 1);
    Serial.print(" hPa");
  } else {
    // MANUAL MODE: Use deployment toggle switch
    deploySwitch.readState();
    bool deployed = deploySwitch.isOn();

    // Set target: fully retracted (0) or fully extended (calibrationMax)
    targetPosition = deployed ? calibrationMax : 0;

    // Print manual mode status
    Serial.print("MANUAL | Deploy Switch: ");
    Serial.print(deployed ? "EXTENDED" : "RETRACTED");
  }

  // Move to target position
  moveToPosition(targetPosition);

  // Print position status
  Serial.print(" | Target: ");
  Serial.print(targetPosition);
  Serial.print(" | Current: ");
  Serial.print(encoderCount);
  Serial.print(" | Error: ");
  Serial.println(targetPosition - encoderCount);

  delay(50);  // Update at 20Hz
}

/*
 * Manual calibration routine with user-controlled max position
 */
void runManualCalibration() {
  Serial.println("=== STARTING MANUAL CALIBRATION ===");
  Serial.println("");

  // Step 1: Move clockwise until limit switch is hit (find MIN)
  Serial.println("Step 1: Finding MIN position (limit switch)...");
  moveClockwise(CALIBRATION_SPEED);

  while (true) {
    limitSwitch.readState();
    if (limitSwitch.isOn()) {  // Limit switch pressed
      stopMotor();
      Serial.println("Limit switch hit!");
      break;
    }
    delay(10);
  }

  delay(500);  // Brief pause

  // Step 2: Zero encoder at limit position (MIN = 0)
  encoderCount = 0;
  calibrationMin = 0;
  Serial.println("MIN position set to 0 (at limit switch)");
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

  // Step 3: User manually positions motor to MAX using deploy toggle
  Serial.println("Step 3: Position motor to desired MAX position");
  Serial.println("  - Toggle Pin 10 ON to move forward (extend)");
  Serial.println("  - Toggle Pin 10 OFF to move backward (retract)");
  Serial.println("  - When satisfied, toggle Pin 4 OFF then ON to save");
  Serial.println("");

  bool lastDeployState = false;
  bool lastModeState = true;  // Start true since we're in calibration mode
  int manualSpeed = 100;  // Slower speed for manual positioning

  while (true) {
    deploySwitch.readState();
    modeSwitch.readState();

    bool currentDeployState = deploySwitch.isOn();
    bool currentModeState = modeSwitch.isOn();

    // Check for mode switch toggle (OFF -> ON) to save
    if (!lastModeState && currentModeState) {
      stopMotor();
      Serial.println("");
      Serial.println("Saving MAX position!");
      break;
    }

    // Control motor based on deploy switch
    if (currentDeployState && !lastDeployState) {
      // Just turned ON - move forward
      Serial.println("Moving forward...");
    } else if (!currentDeployState && lastDeployState) {
      // Just turned OFF - move backward
      Serial.println("Moving backward...");
    }

    if (currentDeployState) {
      moveCounterClockwise(manualSpeed);
    } else {
      moveClockwise(manualSpeed);
    }

    // Print current position periodically
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      Serial.print("Current position: ");
      Serial.println(encoderCount);
      lastPrint = millis();
    }

    lastDeployState = currentDeployState;
    lastModeState = currentModeState;
    delay(10);
  }

  // Step 4: Save calibration
  calibrationMax = encoderCount;

  Serial.println("");
  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.print("MIN position: ");
  Serial.println(calibrationMin);
  Serial.print("MAX position: ");
  Serial.println(calibrationMax);
  Serial.print("Range: ");
  Serial.print((float)calibrationMax / COUNTS_PER_DEGREE, 1);
  Serial.println(" degrees");
  Serial.println("");

  // Save to EEPROM
  saveCalibration();
  Serial.println("Calibration saved to EEPROM");
  Serial.println("");

  isCalibrated = true;
  delay(2000);
}

/*
 * Calculate target position in automatic mode based on IMU and pressure
 * You can adjust the weighting between IMU and pressure as needed
 */
long calculateAutomaticPosition(float pitch, float pressure) {
  // Constrain inputs to expected ranges
  pitch = constrain(pitch, IMU_PITCH_MIN, IMU_PITCH_MAX);
  pressure = constrain(pressure, PRESSURE_MIN, PRESSURE_MAX);

  // Map IMU pitch to position (0 to calibrationMax)
  long pitchPosition = map((long)(pitch * 10),
                           (long)(IMU_PITCH_MIN * 10),
                           (long)(IMU_PITCH_MAX * 10),
                           0,
                           calibrationMax);

  // Map pressure to position (0 to calibrationMax)
  long pressurePosition = map((long)(pressure * 10),
                              (long)(PRESSURE_MIN * 10),
                              (long)(PRESSURE_MAX * 10),
                              0,
                              calibrationMax);

  // Combine both sensors (50/50 weighting - adjust as needed)
  // You could also use just one sensor or different weighting
  long targetPosition = (pitchPosition + pressurePosition) / 2;

  return constrain(targetPosition, 0, calibrationMax);
}

/*
 * Move motor to target position with simple proportional control
 */
void moveToPosition(long targetPosition) {
  long error = targetPosition - encoderCount;
  int deadband = 20;  // Don't move if within 20 counts of target

  if (abs(error) < deadband) {
    stopMotor();
    return;
  }

  // Simple proportional speed control
  int speed = constrain(abs(error) / 2, 80, CONTROL_SPEED);

  if (error > 0) {
    // Need to move counterclockwise (increase count)
    moveCounterClockwise(speed);
  } else {
    // Need to move clockwise (decrease count)
    moveClockwise(speed);
  }
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
 * Save calibration values to EEPROM
 */
void saveCalibration() {
  // Write magic number
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_NUMBER);
  // Write min position
  EEPROM.put(EEPROM_MIN_ADDR, calibrationMin);
  // Write max position
  EEPROM.put(EEPROM_MAX_ADDR, calibrationMax);
  // Write IMU offset
  EEPROM.put(EEPROM_IMU_OFFSET_ADDR, imuPitchOffset);
}

/*
 * Load calibration values from EEPROM
 * Returns true if valid calibration found, false otherwise
 */
bool loadCalibration() {
  // Check magic number
  unsigned long magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic != EEPROM_MAGIC_NUMBER) {
    return false;  // No valid calibration
  }

  // Load calibration values
  EEPROM.get(EEPROM_MIN_ADDR, calibrationMin);
  EEPROM.get(EEPROM_MAX_ADDR, calibrationMax);
  EEPROM.get(EEPROM_IMU_OFFSET_ADDR, imuPitchOffset);

  // Sanity check
  if (calibrationMax <= calibrationMin || calibrationMax > 10000) {
    return false;  // Invalid values
  }

  return true;
}

/*
 * Encoder interrupt service routine
 * Increments/decrements count based on direction
 */
void encoderISR() {
  // Read both encoder channels
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  // Determine direction and update count
  // When moving counterclockwise (increasing angle), count up
  // When moving clockwise (decreasing angle), count down
  if (a == b) {
    encoderCount++;  // Counterclockwise
  } else {
    encoderCount--;  // Clockwise
  }
}
