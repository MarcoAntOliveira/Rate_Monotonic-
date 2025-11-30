#include <Wire.h>
#include <Arduino.h>
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int count = 0;

  Serial.println("Scanning...");

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      count++;
    }
  }

  if (count == 0) Serial.println("Nenhum dispositivo I2C encontrado.");
  else Serial.println("Scan finalizado.");

  delay(1000);
}
