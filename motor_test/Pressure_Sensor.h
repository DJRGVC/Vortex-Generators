#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPRLS.h>

// You can use I2C with the default address 0x18
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin

class Pressure_Sensor {
  public:
    Pressure_Sensor();
    bool begin();
    void readData();
    void printData();

    // Getter for pressure data
    float getPressure_hPa();
    float getPressure_PSI();

  private:
    Adafruit_MPRLS mpr;
    float pressure_hPa;
    float pressure_PSI;
};

#endif
