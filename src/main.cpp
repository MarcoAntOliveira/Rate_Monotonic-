#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <VL53L0X.h>

// Objetos dos sensores
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
VL53L0X sensorVL;

// ----------- TASK DO MAGNETÔMETRO ------------
void TaskMag(void *pvParameters) {
    (void) pvParameters;

    sensors_event_t event;

    for (;;) {
        mag.getEvent(&event);

        float heading = atan2(event.magnetic.y, event.magnetic.x);
        if (heading < 0) heading += 2 * PI;
        float headingDeg = heading * 180.0 / PI;

        Serial.println("---- Magnetômetro ----");
        Serial.printf("X: %.2f  Y: %.2f  Z: %.2f\n",
                      event.magnetic.x, event.magnetic.y, event.magnetic.z);

        Serial.printf("Heading: %.2f°\n", headingDeg);
        Serial.println("------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(300));  // 300ms
    }
}

// ----------- TASK DO VL53L0X ------------
void TaskVL53(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        uint16_t distance = sensorVL.readRangeSingleMillimeters();

        Serial.println("---- VL53L0X ----");
        if (sensorVL.timeoutOccurred()) {
            Serial.println("Timeout!");
        } else {
            Serial.printf("Distância: %d mm\n", distance);
        }
        Serial.println("-----------------\n");

        vTaskDelay(pdMS_TO_TICKS(200)); // 200ms
    }
}

// ----------------------------------------
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    Serial.println("Inicializando sensores...");

    // VL53L0X
    sensorVL.init();
    sensorVL.setTimeout(500);

    // Magnetômetro
    if (!mag.begin()) {
        Serial.println("⚠️ HMC5883L não encontrado!");
        while (1);
    }

    Serial.println("Sensores detectados!\n");

    // Criação das tasks
    xTaskCreatePinnedToCore(
        TaskMag,
        "Task Mag",
        4096,
        NULL,
        1,
        NULL,
        1   // core 1
    );

    xTaskCreatePinnedToCore(
        TaskVL53,
        "Task VL53",
        4096,
        NULL,
        1,
        NULL,
        0   // core 0
    );
}

void loop() {
    // Nada aqui! Tudo é FreeRTOS
    vTaskDelay(pdMS_TO_TICKS(1000));
}
