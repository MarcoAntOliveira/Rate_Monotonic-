#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>

// Objetos dos sensores
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
//VL53L0X sensorVL;
Adafruit_MPU6050 mpu;

// Mutex do I2C
SemaphoreHandle_t i2cMutex;

// ----------- TASK DO MAGNETÔMETRO ------------
void taskMag(void *param){
    while(true){

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        sensors_event_t event;
        mag.getEvent(&event);

        Serial.print("[TASK_MAG] X: ");
        Serial.print(event.magnetic.x);
        Serial.print(" Y: ");
        Serial.print(event.magnetic.y);
        Serial.print(" Z: ");
        Serial.println(event.magnetic.z);
        
        xSemaphoreGive(i2cMutex);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// // ----------- TASK DO VL53L0X ------------
// void taskVL53(void *param){
//     while(true){

//         xSemaphoreTake(i2cMutex, portMAX_DELAY);

//         uint16_t dist = sensorVL.readRangeSingleMillimeters();
        
//         if(sensorVL.timeoutOccurred()){
//             Serial.println("[TASK_VL53] Timeout!");
//         } else {
//             Serial.print("[TASK_VL53] ");
//             Serial.print(dist);
//             Serial.println(" mm");
//         }

//         xSemaphoreGive(i2cMutex);
//         vTaskDelay(200 / portTICK_PERIOD_MS);
//     }
// }

// ----------- TASK DO GY-521 (MPU6050) ------------
void taskGY521 - (void *param){
    sensors_event_t a, g, temp;

    while(true){

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        mpu.getEvent(&a, &g, &temp);

        Serial.print("[TASK_GY521] ACC X: ");
        Serial.print(a.acceleration.x);
        Serial.print(" Y: ");
        Serial.print(a.acceleration.y);
        Serial.print(" Z: ");
        Serial.println(a.acceleration.z);

        Serial.print("[TASK_GY521] GYRO X: ");
        Serial.print(g.gyro.x);
        Serial.print(" Y: ");
        Serial.print(g.gyro.y);
        Serial.print(" Z: ");
        Serial.println(g.gyro.z);

        xSemaphoreGive(i2cMutex);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// ----------- TASK DO GY-521 (MPU6050) ------------
void taskGY521(void *param){
    sensors_event_t a, g, temp;

    while(true){

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        mpu.getEvent(&a, &g, &temp);

        Serial.print("[TASK_GY521] ACC X: ");
        Serial.print(a.acceleration.x);
        Serial.print(" Y: ");
        Serial.print(a.acceleration.y);
        Serial.print(" Z: ");
        Serial.println(a.acceleration.z);

        Serial.print("[TASK_GY521] GYRO X: ");
        Serial.print(g.gyro.x);
        Serial.print(" Y: ");
        Serial.print(g.gyro.y);
        Serial.print(" Z: ");
        Serial.println(g.gyro.z);

        xSemaphoreGive(i2cMutex);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

// ------------------- SETUP -------------------
void setup(){
    Serial.begin(115200);
    Wire.begin(21,22);

    // Inicializa sensores (SEM mutex, pois só ocorre no setup)
    if(!mag.begin()){
        Serial.println("Falha ao iniciar magnetometro!");
    }

    sensorVL.init();
    sensorVL.setTimeout(500);

    if(!mpu.begin()){
        Serial.println("Falha ao iniciar MPU6050!");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // Cria mutex
    i2cMutex = xSemaphoreCreateMutex();

    // Cria tasks
    xTaskCreate(taskMag,   "MagTask",   4096, NULL, 1, NULL);
    //xTaskCreate(taskVL53,  "VL53Task",  4096, NULL, 2, NULL);
    xTaskCreate(taskGY521, "GY521Task", 4096, NULL, 3, NULL);
}

void loop(){
    vTaskDelay(pdMS_TO_TICKS(1000));
}
