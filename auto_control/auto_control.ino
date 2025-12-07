/*
 * Vortex Generator Control with Manual/Automatic Modes
 *
 * This sketch controls vortex generators using a Pololu Micro Metal Gearmotor
 * with encoder and DRV8833 motor driver.
 *
 * MODES:
 * - Manual Mode (Pin 10 toggle OFF): Pin 4 toggle controls deployment
 *   - Pin 4 OFF: Fully retracted (0 degrees)
 *   - Pin 4 ON: Fully extended (65 degrees)
 * - Automatic Mode (Pin 10 toggle ON): IMU/Pressure sensor controls deployment
 *   - Motor position based on IMU roll and pressure readings
 *
 * CALIBRATION MODE:
 * - Calibration runs AUTOMATICALLY at every startup
 * - Calibration process:
 *   1. Auto-move COUNTER-CLOCKWISE until limit switch triggers (MIN = 0)
 *   2. Read IMU roll offset at MIN position (0° angle of attack reference)
 *   3. Read baseline pressure (no airflow reference)
 *   4. Pin 4 ON = move clockwise (extend), OFF = stop
 *   5. Turn Pin 10 ON to save MAX position to EEPROM
 * - Calibration values persist across power cycles
 * - After calibration, system enters selected mode (manual or automatic)
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
 * Mode Toggle Switch (Pin 10):
 *   One terminal -> Arduino Pin 10
 *   Other terminal -> Arduino GND
 *   (OFF = Manual Mode, ON = Automatic Mode)
 *
 * Deployment Toggle Switch (Pin 4):
 *   One terminal -> Arduino Pin 4
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
 *   Pin 4  - Deployment Toggle Switch (Manual mode control)
 *   Pin 5  - Limit Switch
 *   Pin 6  - Motor AIN1 (PWM)
 *   Pin 7  - Encoder A (interrupt)
 *   Pin 8  - Encoder B
 *   Pin 9  - Motor AIN2 (PWM)
 *   Pin 10 - Mode Toggle Switch (Manual/Automatic)
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
#define MODE_SWITCH_PIN 10       // Manual/Automatic mode selection
#define DEPLOY_SWITCH_PIN 4      // Deployment control (manual mode)
#define LIMIT_SWITCH_PIN 5

// EEPROM addresses for calibration storage
#define EEPROM_MAGIC_ADDR 0      // Magic number to verify valid calibration
#define EEPROM_MIN_ADDR 4        // Min position (4 bytes for long)
#define EEPROM_MAX_ADDR 8        // Max position (4 bytes for long)
#define EEPROM_IMU_OFFSET_ADDR 12 // IMU roll offset at 0° angle of attack (4 bytes for float)
#define EEPROM_PRESSURE_BASELINE_ADDR 16 // Baseline pressure for airspeed calc (4 bytes for float)
#define EEPROM_MAGIC_NUMBER 0xCAFEBABE  // Magic number to verify EEPROM data

// Motor control constants
#define CALIBRATION_SPEED 45      // PWM speed during calibration (0-255)
#define CONTROL_SPEED 45          // PWM speed during normal operation (0-255)
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
float imuPitchOffset = 0.0;  // IMU roll at 0° angle of attack
float pressureBaseline = 0.0;  // Baseline pressure (no airflow) in hPa
bool isCalibrated = false;

// IMU roll mapping for automatic mode (relative to calibrated 0°)
#define IMU_PITCH_MIN -30.0   // Angle of attack that maps to motor position 0° (fully retracted)
#define IMU_PITCH_MAX 30.0    // Angle of attack that maps to motor position MAX (fully extended)

// Airspeed mapping for automatic mode (based on pressure difference from baseline)
#define AIRSPEED_MIN 0.0      // Airspeed (m/s) that maps to motor position 0° (fully retracted)
#define AIRSPEED_MAX 20.0     // Airspeed (m/s) that maps to motor position MAX (fully extended)

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

  // Always run calibration at startup
  Serial.println("RUNNING CALIBRATION AT STARTUP");
  Serial.println("");
  runManualCalibration();

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

    float pitchRaw = imuSensor.getRoll();
    float pitch = pitchRaw - imuPitchOffset;  // Apply calibration offset
    float pressure = pressureSensor.getPressure_hPa();
    float airspeed = calculateAirspeed(pressure);  // Calculate airspeed

    // Map sensor readings to motor position (using pitch and airspeed)
    targetPosition = calculateAutomaticPosition(pitch, airspeed);

    // Print automatic mode status
    Serial.print("AUTO | Roll: ");
    Serial.print(pitch, 1);
    Serial.print("° (raw: ");
    Serial.print(pitchRaw, 1);
    Serial.print("°) | Airspeed: ");
    Serial.print(airspeed, 2);
    Serial.print(" m/s | Pressure: ");
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
  Serial.println(F("=== CALIBRATION ==="));

  // Step 1: Move counter-clockwise to limit switch (find MIN)
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

  // Step 2: Zero encoder at limit position (MIN = 0)
  encoderCount = 0;
  calibrationMin = 0;
  Serial.println(F("MIN = 0"));

  // Step 2.5: Read IMU roll at this position (0° angle of attack)
  Serial.println(F("Reading IMU..."));
  delay(500);

  float pitchSum = 0;
  int numReadings = 5;  // Reduced to save memory
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

  // Step 3: User manually positions motor to MAX using deploy toggle
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
      moveClockwise(CALIBRATION_SPEED);
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
  delay(2000);
}

/*
 * Calculate airspeed from pressure difference
 * Uses simplified Bernoulli equation: v = sqrt(2 * ΔP / ρ)
 * where ρ = 1.225 kg/m³ (air density at sea level)
 * Returns airspeed in m/s
 */
float calculateAirspeed(float currentPressure) {
  float deltaPressure_hPa = currentPressure - pressureBaseline;
  float deltaPressure_Pa = deltaPressure_hPa * 100.0;  // Convert hPa to Pa

  // Avoid negative or zero values
  if (deltaPressure_Pa <= 0) {
    return 0.0;
  }

  float airDensity = 1.225;  // kg/m³ at sea level, 15°C
  float airspeed = sqrt((2.0 * deltaPressure_Pa) / airDensity);

  return airspeed;
}

/*
 * Calculate target position in automatic mode based on IMU roll and airspeed
 * You can adjust the weighting between IMU and airspeed as needed
 */
long calculateAutomaticPosition(float pitch, float airspeed) {
  // Constrain inputs to expected ranges
  pitch = constrain(pitch, IMU_PITCH_MIN, IMU_PITCH_MAX);
  airspeed = constrain(airspeed, AIRSPEED_MIN, AIRSPEED_MAX);

  // Map IMU pitch to position (0 to calibrationMax)
  long pitchPosition = map((long)(pitch * 10),
                           (long)(IMU_PITCH_MIN * 10),
                           (long)(IMU_PITCH_MAX * 10),
                           0,
                           calibrationMax);

  // Map airspeed to position (0 to calibrationMax)
  long airspeedPosition = map((long)(airspeed * 10),
                              (long)(AIRSPEED_MIN * 10),
                              (long)(AIRSPEED_MAX * 10),
                              0,
                              calibrationMax);

  // Combine both sensors (50/50 weighting - adjust as needed)
  // You could also use just one sensor or different weighting
  long targetPosition = (pitchPosition + airspeedPosition) / 2;

  return constrain(targetPosition, 0, calibrationMax);
}

/*
 * Move motor to target position with constant speed control
 */
void moveToPosition(long targetPosition) {
  long error = targetPosition - encoderCount;
  int deadband = 20;  // Don't move if within 20 counts of target

  if (abs(error) < deadband) {
    stopMotor();
    return;
  }

  // Use constant speed of 45 for all movements
  if (error > 0) {
    // Need to move counterclockwise (increase count)
    moveCounterClockwise(CONTROL_SPEED);
  } else {
    // Need to move clockwise (decrease count)
    moveClockwise(CONTROL_SPEED);
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
  // Write pressure baseline
  EEPROM.put(EEPROM_PRESSURE_BASELINE_ADDR, pressureBaseline);
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
  EEPROM.get(EEPROM_PRESSURE_BASELINE_ADDR, pressureBaseline);

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
