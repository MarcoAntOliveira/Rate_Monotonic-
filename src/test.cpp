#include <Wire.h>
#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 default no ESP32
  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) Serial.println("No I2C devices found\n");
  else Serial.println("Scan done\n");

  delay(2000);
}
