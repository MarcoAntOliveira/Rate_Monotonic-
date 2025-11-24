#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <VL53L0X.h>

// Objetos dos sensores
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
VL53L0X sensorVL;

// 🔧 DECLARAÇÃO DO MUTEX GLOBAL
SemaphoreHandle_t i2cMutex;

// ----------- TASK DO MAGNETÔMETRO ------------
void taskMag(void *param){
    while(true){

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        sensors_event_t event;
        mag.getEvent(&event);

        xSemaphoreGive(i2cMutex);

        Serial.print("[TASK_MAG] X: ");
        Serial.print(event.magnetic.x);
        Serial.print(" Y: ");
        Serial.print(event.magnetic.y);
        Serial.print(" Z: ");
        Serial.println(event.magnetic.z);

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}



// ------------------- SETUP -------------------
void setup(){
    Serial.begin(115200);
    Wire.begin(21,22);

    mag.begin();
    sensorVL.init();
    sensorVL.setTimeout(500);

    // Criando mutex
    i2cMutex = xSemaphoreCreateMutex();

    // Criando tasks
    xTaskCreate(taskMag,  "MagTask1", 4096, NULL, 1, NULL);
    xTaskCreate(taskMag,  "MagTask2", 4096, NULL, 1, NULL);
    xTaskCreate(taskMag,  "MagTask3", 4096, NULL, 1, NULL);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
