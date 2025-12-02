// Simple I2C Scanner - Use this to find your BNO055
// Upload this, open Serial Monitor at 115200 baud

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("=== I2C Scanner Test ==="));
  Serial.println(F("Leonardo: SDA=A1, SCL=A0"));

  Wire.begin();
  Wire.setClock(100000);  // Slow speed (100kHz) for reliability

  Serial.println(F("Scanning..."));
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println(F("\n--- Scan Start ---"));

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("FOUND: 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);

      // Identify common addresses
      if (address == 0x28 || address == 0x29) {
        Serial.print(F(" <- BNO055!"));
      } else if (address == 0x18) {
        Serial.print(F(" (MPRLS pressure)"));
      } else if (address == 0x68 || address == 0x69) {
        Serial.print(F(" (MPU6050/9250?)"));
      }
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.print(F("ERROR at 0x"));
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  Serial.println(F("--- Scan End ---"));

  if (nDevices == 0) {
    Serial.println(F("NO DEVICES FOUND!"));
    Serial.println(F("Problem: Wiring or power"));
  } else {
    Serial.print(F("Found "));
    Serial.print(nDevices);
    Serial.println(F(" device(s)"));
  }

  delay(5000);  // Scan every 5 seconds
}
