#include "Pressure_Sensor.h"

Pressure_Sensor::Pressure_Sensor() : mpr(RESET_PIN, EOC_PIN) {
  pressure_hPa = 0.0;
  pressure_PSI = 0.0;
}

bool Pressure_Sensor::begin() {
  Serial.println("Initializing MPRLS pressure sensor...");

  if (!mpr.begin()) {
    Serial.println("Failed to initialize MPRLS sensor!");
    Serial.println("Check wiring and I2C address (default 0x18)");
    return false;
  }

  Serial.println("MPRLS sensor initialized successfully!");
  return true;
}

void Pressure_Sensor::readData() {
  pressure_hPa = mpr.readPressure();
  pressure_PSI = pressure_hPa / 68.947572932;  // Convert hPa to PSI
}

void Pressure_Sensor::printData() {
  Serial.print("Pressure:    ");
  Serial.print(pressure_hPa);
  Serial.print(" hPa / ");
  Serial.print(pressure_PSI);
  Serial.println(" PSI");
}

// Getter implementations
float Pressure_Sensor::getPressure_hPa() {
  return pressure_hPa;
}

float Pressure_Sensor::getPressure_PSI() {
  return pressure_PSI;
}
