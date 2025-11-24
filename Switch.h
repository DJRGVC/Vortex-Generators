#ifndef SWITCH_H
#define SWITCH_H

#include <Arduino.h>

class Switch {
  public:
    Switch(int pin);
    void begin();
    void readState();
    void printState();

    // Getter for switch state
    bool isOn();
    bool isOff();

  private:
    int switchPin;
    bool state;  // true = ON (closed), false = OFF (open)
};

#endif
