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

// ----------- TASK DO VL53L0X ------------
void taskVL53(void *param){
    while(true){

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        uint16_t dist = sensorVL.readRangeSingleMillimeters();
        bool timeout = sensorVL.timeoutOccurred();

        xSemaphoreGive(i2cMutex);

        Serial.print("[TASK_VL53] ");

        if(timeout){
            Serial.println("Timeout!");
        } else {
            Serial.print(dist);
            Serial.println(" mm");
        }

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
    xTaskCreate(taskMag,  "MagTask", 4096, NULL, 1, NULL);
    xTaskCreate(taskVL53, "VL53Task", 4096, NULL, 1, NULL);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
