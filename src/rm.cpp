#include <Arduino.h>
#include <Wire.h>
#include "GY521.h"
#include "QMC5883L.h"

QMC5883L mag;
GY521 gyro(0x68);
 const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
// LEDs
#define LED_MAG  2
#define LED_IMU  4
#define LED_GYRO 5

SemaphoreHandle_t i2cMutex;

// ------------------ CONFIG RM ------------------
typedef struct {
    const char* name;
    uint32_t period_ms;
    uint32_t last_exec_us;
    uint32_t worst_exec_us;
} task_info_t;

task_info_t TMag  = {"MAG",  2000, 0, 0};
task_info_t TIMU  = {"IMU",  1000, 0, 0};
task_info_t TGyro = {"GYRO",  500   , 0, 0};

// ------------------ FUNÇÃO RM CHECK ------------------
void checkRM() {
    float U = (TMag.worst_exec_us  / (float)(TMag.period_ms  * 1000)) +
              (TIMU.worst_exec_us  / (float)(TIMU.period_ms  * 1000)) +
              (TGyro.worst_exec_us / (float)(TGyro.period_ms * 1000));

    int n = 3;
    float UB = n * (pow(2.0, 1.0/n) - 1.0);

    Serial.println("\n=== TESTE RM ===");
    Serial.printf("U  = %.4f\n", U);
    Serial.printf("UB = %.4f\n", UB);

    if (U <= UB)
        Serial.println("🟢 RM: Sistema escalonável");
    else
        Serial.println("🔴 RM: Sistema NÃO escalonável");
}

// =======================================================
//                TAREFA MAG
// =======================================================
void taskMag(void *p) {
    pinMode(LED_MAG, OUTPUT);
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        uint64_t start = esp_timer_get_time();
        digitalWrite(LED_MAG, HIGH);

        xSemaphoreTake(i2cMutex, portMAX_DELAY);
            int x, y, z;
            mag.read(x, y, z);
            Serial.printf("MAG %d %d %d\n", x, y, z);
            digitalWrite(LED_MAG, LOW);
        xSemaphoreGive(i2cMutex);

    

        uint64_t end = esp_timer_get_time();
        uint32_t exec = end - start;
        if (exec > TMag.worst_exec_us) TMag.worst_exec_us = exec;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TMag.period_ms));
    }
}

// =======================================================
//                TAREFA IMU
// =======================================================
void taskIMU(void *p) {
    pinMode(LED_IMU, OUTPUT);
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        uint64_t start = esp_timer_get_time();
        digitalWrite(LED_IMU, HIGH);

        xSemaphoreTake(i2cMutex, portMAX_DELAY);
            gyro.read();
           
            float pitch = gyro.getPitch();
            float roll  = gyro.getRoll();
            float yaw   = gyro.getYaw();
            Serial.printf("IMU %.2f %.2f %.2f\n", pitch, roll, yaw);
            digitalWrite(LED_GYRO, LOW);
        xSemaphoreGive(i2cMutex);

  

        uint64_t end = esp_timer_get_time();
        uint32_t exec = end - start;
        TIMU.last_exec_us = exec;
        if (exec > TIMU.worst_exec_us) TIMU.worst_exec_us = exec;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TIMU.period_ms));
    }
}

// =======================================================
//                TAREFA GYRO
// =======================================================
void taskGyro(void *p) {
    pinMode(LED_GYRO, OUTPUT);
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        uint64_t start = esp_timer_get_time();
        digitalWrite(LED_GYRO, HIGH);

        xSemaphoreTake(i2cMutex, portMAX_DELAY);
            gyro.read();
           
            float x = gyro.getAngleX();
            float y = gyro.getAngleY();
            float z = gyro.getAngleZ();
            Serial.printf("GYRO %.2f %.2f %.2f\n", x, y, z);
            digitalWrite(LED_GYRO, LOW);
        xSemaphoreGive(i2cMutex);

      

        uint64_t end = esp_timer_get_time();
        uint32_t exec = end - start;
        TGyro.last_exec_us = exec;
        if( exec < TGyro.period_ms) 
        if (exec > TGyro.worst_exec_us) TGyro.worst_exec_us = exec;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TGyro.period_ms));
        
    }
}

// =======================================================
//                    SETUP
// =======================================================
void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);

    mag.begin(Wire);

    while (!gyro.wakeup()) {
        Serial.println("Falha IMU!");
        delay(1000);
    }

    gyro.setAccelSensitivity(2);
    gyro.setGyroSensitivity(1);

    i2cMutex = xSemaphoreCreateMutex();

    // Prioridades RM (menor período = maior prioridade)
    xTaskCreate(taskGyro, "GYRO", 4096, NULL, 3, NULL);
    xTaskCreate(taskIMU,  "IMU",  4096, NULL, 2, NULL);
    xTaskCreate(taskMag,  "MAG",  4096, NULL, 1, NULL);

    Serial.println("\nSistema iniciado. Medindo tempos...");
    delay(5000);
    checkRM();  // faz análise RM após alguns segundos
}

void loop() {}
