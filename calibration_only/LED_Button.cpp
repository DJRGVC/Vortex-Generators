#include "LED_Button.h"

LED_Button::LED_Button(int btnPin, int rPin, int gPin, int bPin) {
  buttonPin = btnPin;
  redPin = rPin;
  greenPin = gPin;
  bluePin = bPin;
  buttonState = false;
  currentColor = LED_OFF;
  redValue = greenValue = blueValue = 0;
}

void LED_Button::begin() {
  // Initialize button pin with pull-up
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Turn off LED initially
  turnOff();

  Serial.print("LED Button initialized on pin ");
  Serial.print(buttonPin);
  Serial.print(" (LED: R=");
  Serial.print(redPin);
  Serial.print(" G=");
  Serial.print(greenPin);
  Serial.print(" B=");
  Serial.print(bluePin);
  Serial.println(")");
}

void LED_Button::readState() {
  // Read the button state
  // With INPUT_PULLUP: LOW = pressed, HIGH = released
  int reading = digitalRead(buttonPin);
  buttonState = (reading == LOW);
}

void LED_Button::printState() {
  Serial.print("Button:      ");
  if (buttonState) {
    Serial.print("PRESSED");
  } else {
    Serial.print("RELEASED");
  }

  Serial.print("  |  LED: ");
  switch(currentColor) {
    case LED_OFF:
      Serial.println("OFF");
      break;
    case LED_RED:
      Serial.println("RED");
      break;
    case LED_GREEN:
      Serial.println("GREEN");
      break;
    case LED_BLUE:
      Serial.println("BLUE");
      break;
    case LED_YELLOW:
      Serial.println("YELLOW");
      break;
    case LED_WHITE:
      Serial.println("WHITE");
      break;
    case LED_CYAN:
      Serial.println("CYAN");
      break;
    case LED_MAGENTA:
      Serial.println("MAGENTA");
      break;
    case LED_CUSTOM:
      Serial.print("CUSTOM (R:");
      Serial.print(redValue);
      Serial.print(" G:");
      Serial.print(greenValue);
      Serial.print(" B:");
      Serial.print(blueValue);
      Serial.println(")");
      break;
  }
}

void LED_Button::setColor(LEDColor color) {
  currentColor = color;

  switch(color) {
    case LED_OFF:
      redValue = greenValue = blueValue = 0;
      break;
    case LED_RED:
      redValue = 255; greenValue = 0; blueValue = 0;
      break;
    case LED_GREEN:
      redValue = 0; greenValue = 255; blueValue = 0;
      break;
    case LED_BLUE:
      redValue = 0; greenValue = 0; blueValue = 255;
      break;
    case LED_YELLOW:
      redValue = 255; greenValue = 255; blueValue = 0;
      break;
    case LED_WHITE:
      redValue = 255; greenValue = 255; blueValue = 255;
      break;
    case LED_CYAN:
      redValue = 0; greenValue = 255; blueValue = 255;
      break;
    case LED_MAGENTA:
      redValue = 255; greenValue = 0; blueValue = 255;
      break;
    case LED_CUSTOM:
      // Keep current custom values
      break;
  }

  applyColor();
}

void LED_Button::setCustomColor(int red, int green, int blue) {
  currentColor = LED_CUSTOM;
  redValue = constrain(red, 0, 255);
  greenValue = constrain(green, 0, 255);
  blueValue = constrain(blue, 0, 255);
  applyColor();
}

void LED_Button::turnOff() {
  currentColor = LED_OFF;
  redValue = greenValue = blueValue = 0;
  applyColor();
}

void LED_Button::turnOn() {
  // Turn on with last non-off color, or default to white
  if (currentColor == LED_OFF) {
    setColor(LED_WHITE);
  } else {
    applyColor();
  }
}

void LED_Button::applyColor() {
  // Write PWM values to LED pins
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

// Getters
bool LED_Button::isPressed() {
  return buttonState;
}

bool LED_Button::isReleased() {
  return !buttonState;
}

LEDColor LED_Button::getCurrentColor() {
  return currentColor;
}
