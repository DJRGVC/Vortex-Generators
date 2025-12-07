/*
 * Simple Motor Test - Verify motor can move
 *
 * This sketch tests basic motor movement using two toggle switches.
 *
 * CONTROLS:
 * - Both switches OFF: Motor stopped
 * - Pin 4 ON (Pin 10 OFF): Motor moves CLOCKWISE (left)
 * - Pin 10 ON (Pin 4 OFF): Motor moves COUNTERCLOCKWISE (right)
 * - Both switches ON: Motor stopped (safety)
 *
 * WIRING:
 * DRV8833 Motor Driver:
 *   VM   -> Arduino 5V (motor power)
 *   VCC  -> Arduino 5V (logic power)
 *   GND  -> Arduino GND
 *   SLP  -> Arduino 5V (keep driver awake)
 *   AIN1 -> Arduino Pin 6
 *   AIN2 -> Arduino Pin 9
 *
 * Motor (to DRV8833):
 *   Red wire   -> AOUT1
 *   Black wire -> AOUT2
 *
 * Switches:
 *   Pin 4  -> One terminal to Pin 4, other to GND
 *   Pin 10 -> One terminal to Pin 10, other to GND
 */

// Motor driver pins
#define MOTOR_AIN1 6
#define MOTOR_AIN2 9

// Switch pins
#define SWITCH_PIN_4 4
#define SWITCH_PIN_10 10

// Very slow speed for testing (0-255)
#define TEST_SPEED 45  // Start very slow, increase if needed

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("=== MOTOR TEST ===");
  Serial.println("Testing basic motor movement");
  Serial.println("");

  // Setup motor pins as outputs
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);

  // Setup switch pins with pullups
  pinMode(SWITCH_PIN_4, INPUT_PULLUP);
  pinMode(SWITCH_PIN_10, INPUT_PULLUP);

  // Start with motor stopped
  stopMotor();

  Serial.println("Ready!");
  Serial.println("Pin 4 ON  = Clockwise");
  Serial.println("Pin 10 ON = Counterclockwise");
  Serial.println("");
}

void loop() {
  // Read switches (LOW = pressed/ON because of pullup)
  bool switch4 = !digitalRead(SWITCH_PIN_4);  // Invert because of pullup
  bool switch10 = !digitalRead(SWITCH_PIN_10);

  // Determine motor action based on switches
  if (switch4 && !switch10) {
    // Pin 4 ON, Pin 10 OFF -> Clockwise
    moveClockwise();
    Serial.println("Moving CLOCKWISE");
  }
  else if (switch10 && !switch4) {
    // Pin 10 ON, Pin 4 OFF -> Counterclockwise
    moveCounterClockwise();
    Serial.println("Moving COUNTERCLOCKWISE");
  }
  else {
    // Both OFF or both ON -> Stop
    stopMotor();
    if (switch4 && switch10) {
      Serial.println("Both switches ON - STOPPED (safety)");
    }
  }

  delay(100);  // Small delay for readability
}

void moveClockwise() {
  analogWrite(MOTOR_AIN1, TEST_SPEED);
  analogWrite(MOTOR_AIN2, 0);
}

void moveCounterClockwise() {
  analogWrite(MOTOR_AIN1, 0);
  analogWrite(MOTOR_AIN2, TEST_SPEED);
}

void stopMotor() {
  analogWrite(MOTOR_AIN1, 0);
  analogWrite(MOTOR_AIN2, 0);
}
