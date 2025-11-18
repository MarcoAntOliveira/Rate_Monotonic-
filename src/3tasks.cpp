#include <Arduino.h>
#include <Wire.h>

// --- VL53L0X ---
#include "VL53L0X.h"
VL53L0X sensor;

// --- HMC5883L ---
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// --- MPU6050 ---
#include "MPU6050.h"
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Serial.println("Iniciando sensores...");

  sensor.init();
  sensor.setTimeout(500);
  Serial.println("VL53L0X iniciado!");

  // ---- HMC5883L ----
  if (!mag.begin()) {
    Serial.println("Falha ao iniciar HMC5883L!");
    while (1);
  }
  Serial.println("HMC5883L iniciado!");

  // ---- MPU6050 ----
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Falha ao iniciar MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 iniciado!");
}

void loop() {

  // ----- VL53L0X -----
  if (sensor.timeoutOccurred()) {
    Serial.println("Timeout");
  } else {
    //Serial.print("Distancia: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

    delay(200);




  // ----- HMC5883L -----
  sensors_event_t event;
  mag.getEvent(&event);
  Serial.print("Mag X: "); Serial.print(event.magnetic.x);
  Serial.print("  Y: "); Serial.print(event.magnetic.y);
  Serial.print("  Z: "); Serial.println(event.magnetic.z);

  // ----- MPU6050 -----
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" Y: "); Serial.print(ay);
  Serial.print(" Z: "); Serial.println(az);

  Serial.print("Gyro  X: "); Serial.print(gx);
  Serial.print(" Y: "); Serial.print(gy);
  Serial.print(" Z: "); Serial.println(gz);

  Serial.println("---------------------------");
  delay(400);
}
