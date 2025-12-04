#include "IMU_Sensor.h"

IMU_Sensor::IMU_Sensor() : bno(Adafruit_BNO055(55)) {
  heading = roll = pitch = 0.0;
  gyroX = gyroY = gyroZ = 0.0;
  accelX = accelY = accelZ = 0.0;
}

bool IMU_Sensor::begin() {
  Serial.println(F("=== BNO055 Init ==="));

  // Common I2C addresses to try
  uint8_t addresses[] = {0x28, 0x29, 0x18, 0x19, 0x68, 0x69};
  int numAddresses = 6;
  bool found = false;

  for (int i = 0; i < numAddresses; i++) {
    uint8_t addr = addresses[i];
    Serial.print(F("Try 0x"));
    Serial.print(addr, HEX);
    Serial.print(F("..."));

    // Check if device responds at this address
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F(" I2C OK, "));

      // Try to initialize BNO055 at this address
      Adafruit_BNO055 testBno = Adafruit_BNO055(55, addr);
      if (testBno.begin()) {
        Serial.println(F("BNO055 YES!"));
        bno = testBno;
        found = true;
        break;
      } else {
        Serial.println(F("not BNO055"));
      }
    } else {
      Serial.print(F(" I2C err "));
      Serial.println(error);
    }
    delay(50);
  }

  if (!found) {
    Serial.println(F("ERROR: No BNO055!"));
    Serial.println(F("Check wiring & power"));
    return false;
  }

  delay(100);
  uint8_t sysStatus, selfTestResult, sysError;
  bno.getSystemStatus(&sysStatus, &selfTestResult, &sysError);

  if (sysError != 0) {
    Serial.print(F("SysErr: 0x"));
    Serial.println(sysError, HEX);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  // Check calibration status
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(F("Cal: S"));
  Serial.print(system);
  Serial.print(F(" G"));
  Serial.print(gyro);
  Serial.print(F(" A"));
  Serial.print(accel);
  Serial.print(F(" M"));
  Serial.println(mag);

  Serial.println(F("BNO055 OK"));
  return true;
}

void IMU_Sensor::readData() {
  // Get orientation data (Euler angles)
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  heading = orientationData.orientation.x;
  roll = orientationData.orientation.y;
  pitch = orientationData.orientation.z;

  // Check for valid orientation data
  if (isnan(heading) || isnan(roll) || isnan(pitch)) {
    Serial.println(F("WARN: Orientation NaN"));
  }

  // Get angular velocity
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyroX = gyroscope.x();
  gyroY = gyroscope.y();
  gyroZ = gyroscope.z();

  // Check for valid gyroscope data
  if (isnan(gyroX) || isnan(gyroY) || isnan(gyroZ)) {
    Serial.println(F("WARN: Gyro NaN"));
  }

  // Get acceleration
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accelX = accelerometer.x();
  accelY = accelerometer.y();
  accelZ = accelerometer.z();

  // Check for valid accelerometer data
  if (isnan(accelX) || isnan(accelY) || isnan(accelZ)) {
    Serial.println(F("WARN: Accel NaN"));
  }

  // Periodically check system status (every ~10 seconds)
  static int readCount = 0;
  readCount++;
  if (readCount >= 100) {
    readCount = 0;
    uint8_t sysStatus, selfTestResult, sysError;
    bno.getSystemStatus(&sysStatus, &selfTestResult, &sysError);

    if (sysError != 0) {
      Serial.print(F("Err: 0x"));
      Serial.println(sysError, HEX);
    }

    // Check calibration status
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print(F("Cal S"));
    Serial.print(system);
    Serial.print(F(" G"));
    Serial.print(gyro);
    Serial.print(F(" A"));
    Serial.print(accel);
    Serial.print(F(" M"));
    Serial.println(mag);
  }
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

bool IMU_Sensor::checkConnection() {
  uint8_t sysStatus, selfTestResult, sysError;
  bno.getSystemStatus(&sysStatus, &selfTestResult, &sysError);

  if (sysStatus == 0xFF && sysError == 0xFF) {
    Serial.println(F("ERR: BNO055 lost!"));
    return false;
  }

  if (sysError != 0) {
    Serial.print(F("WARN: Err 0x"));
    Serial.println(sysError, HEX);
    return false;
  }

  return true;
}

void IMU_Sensor::printDiagnostics() {
  Serial.println(F("=== BNO055 Diag ==="));

  uint8_t sysStatus, selfTestResult, sysError;
  bno.getSystemStatus(&sysStatus, &selfTestResult, &sysError);

  Serial.print(F("SysStat: 0x"));
  Serial.println(sysStatus, HEX);
  Serial.print(F("SysErr: 0x"));
  Serial.println(sysError, HEX);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(F("Cal: S"));
  Serial.print(system);
  Serial.print(F(" G"));
  Serial.print(gyro);
  Serial.print(F(" A"));
  Serial.print(accel);
  Serial.print(F(" M"));
  Serial.println(mag);

  Serial.print(F("Temp: "));
  Serial.print(bno.getTemp());
  Serial.println(F("C"));
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
