#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Inicializando MPU6050...");

  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar MPU6050!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU6050 encontrado!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Aceleração X: ");
  Serial.print(a.acceleration.x);
  Serial.print("  Y: ");
  Serial.print(a.acceleration.y);
  Serial.print("  Z: ");
  Serial.println(a.acceleration.z);

  Serial.print("Giro X: ");
  Serial.print(g.gyro.x);
  Serial.print("  Y: ");
  Serial.print(g.gyro.y);
  Serial.print("  Z: ");
  Serial.println(g.gyro.z);

  Serial.println("----------------------");
  delay(500);
}
