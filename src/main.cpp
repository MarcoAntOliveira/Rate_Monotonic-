#include <Arduino.h>
#include <Wire.h>
#include "GY521.h"
#include "QMC5883L.h"

//  Sensores
QMC5883L mag;
GY521 gyro(0x68);

// GPIOs
#define LED_MAG  2
#define LED_IMU  4
#define LED_GYRO 5

// Mutex para o I2C
SemaphoreHandle_t i2cMutex;


// ------------------ TASK MAGNETÔMETRO ------------------
void taskMag(void *param) {
    pinMode(LED_MAG, OUTPUT);

    while (true) {
        digitalWrite(LED_MAG, HIGH);

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        int x, y, z;
        if (mag.read(x, y, z)) {
            Serial.printf("MAG %d %d %d\n", x, y, z);
        }

        xSemaphoreGive(i2cMutex);

        digitalWrite(LED_MAG, LOW);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



// ------------------ TASK IMU – Pitch/Roll/Yaw ------------------
void taskIMU(void *param) {
    pinMode(LED_IMU, OUTPUT);

    while (true) {
        digitalWrite(LED_IMU, HIGH);

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        gyro.read();
        float pitch = gyro.getPitch();
        float roll  = gyro.getRoll();
        float yaw   = gyro.getYaw();

        Serial.printf("IMU %.2f %.2f %.2f\n", pitch, roll, yaw);

        xSemaphoreGive(i2cMutex);

        digitalWrite(LED_IMU, LOW);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



// ------------------ TASK GYRO — getAngleX/Y/Z ------------------
void taskGyro(void *param) {
    pinMode(LED_GYRO, OUTPUT);

    while (true) {
        digitalWrite(LED_GYRO, HIGH);

        xSemaphoreTake(i2cMutex, portMAX_DELAY);

        gyro.read();
        float x = gyro.getAngleX();
        float y = gyro.getAngleY();
        float z = gyro.getAngleZ();

        Serial.printf("GYRO %.2f %.2f %.2f\n", x, y, z);

        xSemaphoreGive(i2cMutex);

        digitalWrite(LED_GYRO, LOW);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



// ----------------------- SETUP -----------------------
void setup() {
    Serial.begin(115200);
    delay(300);

    Wire.begin(21, 22);

    // Inicializa QMC5883L
    if (!mag.begin(Wire)) {
        Serial.println("ERRO QMC5883L!");
        while (1);
    }

    // Inicializa GY-521
    while (!gyro.wakeup()) {
        Serial.println("ERRO GY521!");
        delay(1000);
    }

    gyro.setAccelSensitivity(2);
    gyro.setGyroSensitivity(1);
    gyro.setThrottle();

    Serial.println("SENSORES OK — Criando tasks...");

    i2cMutex = xSemaphoreCreateMutex();

    xTaskCreate(taskMag,  "MagTask",   4096, NULL, 1, NULL);
    xTaskCreate(taskIMU,  "IMUTask",   4096, NULL, 2, NULL);
    xTaskCreate(taskGyro, "GyroTask",  4096, NULL, 3, NULL);
}



// ------------------------ LOOP ------------------------
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
