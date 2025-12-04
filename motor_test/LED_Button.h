#ifndef LED_BUTTON_H
#define LED_BUTTON_H

#include <Arduino.h>

// Predefined LED colors
enum LEDColor {
  LED_OFF,
  LED_RED,
  LED_GREEN,
  LED_BLUE,
  LED_YELLOW,
  LED_WHITE,
  LED_CYAN,
  LED_MAGENTA,
  LED_CUSTOM
};

class LED_Button {
  public:
    // Constructor: buttonPin, redPin, greenPin, bluePin
    LED_Button(int btnPin, int rPin, int gPin, int bPin);

    void begin();
    void readState();
    void printState();

    // LED control methods
    void setColor(LEDColor color);
    void setCustomColor(int red, int green, int blue);  // RGB values 0-255
    void turnOff();
    void turnOn();  // Turn on with last set color

    // Getters
    bool isPressed();
    bool isReleased();
    LEDColor getCurrentColor();

  private:
    int buttonPin;
    int redPin, greenPin, bluePin;
    bool buttonState;  // true = pressed, false = released
    LEDColor currentColor;
    int redValue, greenValue, blueValue;  // Current RGB values (0-255)

    void applyColor();  // Internal method to apply RGB values to pins
};

#endif
