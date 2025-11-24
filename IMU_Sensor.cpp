#include "IMU_Sensor.h"

IMU_Sensor::IMU_Sensor() : bno(Adafruit_BNO055(55)) {
  heading = roll = pitch = 0.0;
  gyroX = gyroY = gyroZ = 0.0;
  accelX = accelY = accelZ = 0.0;
}

bool IMU_Sensor::begin() {
  Serial.println("Initializing BNO055 IMU...");

  // Try to initialize the BNO055 sensor with default address (0x28)
  if (!bno.begin()) {
    Serial.println("Failed at 0x28. Trying address 0x29...");
    // Try alternative address
    Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29);
    if (!bno2.begin()) {
      Serial.println("No BNO055 detected at either address!");
      return false;
    } else {
      Serial.println("BNO055 found at address 0x29!");
      bno = bno2;
    }
  } else {
    Serial.println("BNO055 found at address 0x28!");
  }

  delay(1000);

  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 initialized successfully!");
  return true;
}

void IMU_Sensor::readData() {
  // Get orientation data (Euler angles)
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  heading = orientationData.orientation.x;
  roll = orientationData.orientation.y;
  pitch = orientationData.orientation.z;

  // Get angular velocity
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyroX = gyroscope.x();
  gyroY = gyroscope.y();
  gyroZ = gyroscope.z();

  // Get acceleration
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accelX = accelerometer.x();
  accelY = accelerometer.y();
  accelZ = accelerometer.z();
}

void IMU_Sensor::printData() {
  Serial.print("Orientation: ");
  Serial.print("Heading=");
  Serial.print(heading);
  Serial.print(" Roll=");
  Serial.print(roll);
  Serial.print(" Pitch=");
  Serial.print(pitch);
  Serial.println();

  Serial.print("Gyroscope:   ");
  Serial.print("X=");
  Serial.print(gyroX);
  Serial.print(" Y=");
  Serial.print(gyroY);
  Serial.print(" Z=");
  Serial.print(gyroZ);
  Serial.println(" rad/s");

  Serial.print("Accel:       ");
  Serial.print("X=");
  Serial.print(accelX);
  Serial.print(" Y=");
  Serial.print(accelY);
  Serial.print(" Z=");
  Serial.print(accelZ);
  Serial.println(" m/s^2");
}

// Getter implementations
float IMU_Sensor::getHeading() { return heading; }
float IMU_Sensor::getRoll() { return roll; }
float IMU_Sensor::getPitch() { return pitch; }
float IMU_Sensor::getGyroX() { return gyroX; }
float IMU_Sensor::getGyroY() { return gyroY; }
float IMU_Sensor::getGyroZ() { return gyroZ; }
float IMU_Sensor::getAccelX() { return accelX; }
float IMU_Sensor::getAccelY() { return accelY; }
float IMU_Sensor::getAccelZ() { return accelZ; }
