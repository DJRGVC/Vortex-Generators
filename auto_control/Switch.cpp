#include "Switch.h"

Switch::Switch(int pin) {
  switchPin = pin;
  state = false;
}

void Switch::begin() {
  pinMode(switchPin, INPUT_PULLUP);
  Serial.print("Switch initialized on pin ");
  Serial.println(switchPin);
}

void Switch::readState() {
  // Read the pin state
  // With INPUT_PULLUP: LOW = switch closed (ON), HIGH = switch open (OFF)
  int reading = digitalRead(switchPin);
  state = (reading == LOW);  // true if switch is ON (closed)
}

void Switch::printState() {
  Serial.print("Switch:      ");
  if (state) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }
}

// Getter implementations
bool Switch::isOn() {
  return state;
}

bool Switch::isOff() {
  return !state;
}
