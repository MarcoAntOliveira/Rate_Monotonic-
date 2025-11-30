#include "GY521.h"

GY521 sensor(0x68);

uint32_t counter = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("GY521_LIB_VERSION: ");
  Serial.println(GY521_LIB_VERSION);

  // PINOS I2C DO ESP32
  Wire.begin(21, 22);   // <<< ESSENCIAL

  delay(100);
  while (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521: please check the GY521 address (0x68/0x69)");
    delay(1000);
  }

  sensor.setAccelSensitivity(2);  // 8 g
  sensor.setGyroSensitivity(1);   // 500 °/s

  sensor.setThrottle();
  Serial.println("start...");

  sensor.axe = 0.574;
  sensor.aye = -0.002;
  sensor.aze = -1.043;
  sensor.gxe = 10.702;
  sensor.gye = -6.436;
  sensor.gze = -0.676;
}

void loop()
{
  sensor.read();
  float x = sensor.getAngleX();
  float y = sensor.getAngleY();
  float z = sensor.getAngleZ();

  Serial.print(x, 1);
  Serial.print('\t');
  Serial.print(y, 1);
  Serial.print('\t');
  Serial.print(z, 1);
  Serial.println();

  counter++;
}
