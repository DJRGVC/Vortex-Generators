#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU_Sensor {
  public:
    IMU_Sensor();
    bool begin();
    void readData();
    void printData();

    // Getters for sensor data
    float getHeading();
    float getRoll();
    float getPitch();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getAccelX();
    float getAccelY();
    float getAccelZ();

  private:
    Adafruit_BNO055 bno;

    // Sensor data storage
    float heading, roll, pitch;
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
};

#endif
