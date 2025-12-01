  /*
 * Motor Control with Encoder and IMU Pitch Control
 *
 * This sketch controls a Pololu Micro Metal Gearmotor with encoder
 * using a DRV8833 motor driver.
 *
 * Calibration sequence:
 * 1. Move clockwise until limit switch is hit
 * 2. Rotate 65 degrees counterclockwise to find upper limit
 * 3. Control motor angle based on IMU pitch within this range
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
 * Toggle Switch:
 *   One terminal -> Arduino Pin 4
 *   Other terminal -> Arduino GND
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
 *   Pin 4  - Toggle Switch
 *   Pin 5  - Limit Switch
 *   Pin 6  - Motor AIN1 (PWM)
 *   Pin 7  - Encoder A (interrupt)
 *   Pin 8  - Encoder B
 *   Pin 9  - Motor AIN2 (PWM)
 *   Pin A0 - I2C SCL (IMU & Pressure)
 *   Pin A1 - I2C SDA (IMU & Pressure)
 * ============================================================================
 */

#include <Wire.h>
#include "IMU_Sensor.h"
#include "Switch.h"

// Motor driver pins
#define MOTOR_AIN1 6
#define MOTOR_AIN2 9

// Encoder pins (Pin 7 has interrupt capability on Leonardo)
#define ENCODER_A 7
#define ENCODER_B 8

// Limit switch pin
#define LIMIT_SWITCH_PIN 5

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
long calibrationMax = 0;  // Maximum position (65 degrees from limit)
bool isCalibrated = false;

// IMU pitch mapping (adjust these to your desired range)
#define IMU_PITCH_MIN -30.0   // IMU pitch that maps to motor position 0°
#define IMU_PITCH_MAX 30.0    // IMU pitch that maps to motor position 65°

// Create sensor objects
IMU_Sensor imuSensor;
Switch limitSwitch(LIMIT_SWITCH_PIN);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Motor Control System Starting...");
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

  // Initialize limit switch
  limitSwitch.begin();

  Serial.println("All systems initialized!");
  Serial.println("");
  delay(1000);

  // Run calibration
  calibrateMotor();
}

void loop() {
  if (!isCalibrated) {
    Serial.println("ERROR: Motor not calibrated!");
    delay(1000);
    return;
  }

  // Read IMU data
  imuSensor.readData();
  float pitch = imuSensor.getPitch();

  // Map IMU pitch to motor position (0 to calibrationMax counts)
  long targetPosition = mapPitchToPosition(pitch);

  // Move to target position
  moveToPosition(targetPosition);

  // Print status
  Serial.print("Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("°  |  Target Pos: ");
  Serial.print(targetPosition);
  Serial.print("  |  Current Pos: ");
  Serial.print(encoderCount);
  Serial.print("  |  Error: ");
  Serial.println(targetPosition - encoderCount);

  delay(50);  // Update at 20Hz
}

/*
 * Calibration routine
 */
void calibrateMotor() {
  Serial.println("=== STARTING CALIBRATION ===");
  Serial.println("");

  // Step 1: Move clockwise until limit switch is hit
  Serial.println("Step 1: Moving clockwise to find limit switch...");
  moveClockwise(CALIBRATION_SPEED);

  while (true) {
    limitSwitch.readState();
    if (limitSwitch.isOn()) {  // Limit switch pressed (reads LOW, but isOn() returns true)
      stopMotor();
      Serial.println("Limit switch hit!");
      break;
    }
    delay(10);
  }

  delay(500);  // Brief pause

  // Step 2: Zero encoder at limit position
  encoderCount = 0;
  Serial.println("Encoder position zeroed at limit.");
  Serial.println("");

  // Step 3: Move counterclockwise 65 degrees
  Serial.print("Step 2: Moving ");
  Serial.print(CALIBRATION_DEGREES, 0);
  Serial.println(" degrees counterclockwise...");

  long targetCounts = (long)(CALIBRATION_DEGREES * COUNTS_PER_DEGREE);
  Serial.print("Target counts: ");
  Serial.println(targetCounts);

  moveCounterClockwise(CALIBRATION_SPEED);

  while (encoderCount < targetCounts) {
    // Check if we accidentally hit the limit switch
    limitSwitch.readState();
    if (limitSwitch.isOn()) {
      stopMotor();
      Serial.println("ERROR: Hit limit switch during calibration!");
      while(1);  // Halt
    }

    // Print progress
    if (encoderCount % 200 == 0) {
      Serial.print("Progress: ");
      Serial.print(encoderCount);
      Serial.print(" / ");
      Serial.println(targetCounts);
    }
    delay(10);
  }

  stopMotor();
  Serial.println("Reached target position!");
  Serial.println("");

  // Step 4: Store calibration
  calibrationMax = encoderCount;
  isCalibrated = true;

  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.print("Working range: 0 to ");
  Serial.print(calibrationMax);
  Serial.println(" counts");
  Serial.print("This represents 0 to ");
  Serial.print(CALIBRATION_DEGREES, 1);
  Serial.println(" degrees");
  Serial.println("");

  delay(2000);
}

/*
 * Map IMU pitch to motor position
 */
long mapPitchToPosition(float pitch) {
  // Constrain pitch to expected range
  pitch = constrain(pitch, IMU_PITCH_MIN, IMU_PITCH_MAX);

  // Map pitch to position (0 to calibrationMax)
  long position = map((long)(pitch * 10),
                      (long)(IMU_PITCH_MIN * 10),
                      (long)(IMU_PITCH_MAX * 10),
                      0,
                      calibrationMax);

  return constrain(position, 0, calibrationMax);
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
