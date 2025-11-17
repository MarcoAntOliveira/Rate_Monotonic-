#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <VL53L0X.h>

// Cria o objeto do sensor (ID arbitrário 12345)
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
VL53L0X sensor;
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // Pinos SDA e SCL do ESP32

  Serial.println("Inicializando HMC5883L...");
  sensor.init();
  sensor.setTimeout(500);
  if (!mag.begin()) {
    Serial.println("⚠️  Sensor HMC5883L não encontrado!");
    while (1); // Para tudo se não detectar o sensor
  }

  Serial.println("✅ HMC5883L detectado com sucesso!");
  delay(1000);
}

void loop() {
  sensors_event_t event;
  mag.getEvent(&event);
  uint16_t distance = sensor.readRangeSingleMillimeters();

  Serial.println("---- Leitura do Magnetômetro ----");
  Serial.print("Mag X: "); Serial.print(event.magnetic.x);
  Serial.print(" | Y: "); Serial.print(event.magnetic.y);
  Serial.print(" | Z: "); Serial.println(event.magnetic.z);

  // Calcula direção (heading) em graus
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  if (heading < 0) heading += 2 * PI;
  float headingDegrees = heading * 180 / PI;

  Serial.print("Direção (heading): ");
  Serial.print(headingDegrees);
  Serial.println(" °");

  if (sensor.timeoutOccurred()) {
    Serial.println("Timeout");
  } else {
    //Serial.print("Distancia: ");
    Serial.print(distance);
    Serial.println(" mm");
  }

    delay(200);

  Serial.println("-----------------------------\n");
  delay(1000);
}
