#include <Arduino.h>
#include <Wire.h>
#include "GY521.h"
#include "QMC5883L.h"

QMC5883L mag;
GY521 gyro(0x68);

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
task_info_t TGyro = {"GYRO",  500, 0, 0};



// =======================================================
//                TAREFA MAG
// =======================================================
void taskMag(void *p) {
    pinMode(LED_MAG, OUTPUT);
    TickType_t lastWake = xTaskGetTickCount();

    int x, y, z;
    uint64_t start, end;

    while (true) {
        digitalWrite(LED_MAG, HIGH);
        
        // --- SEÇÃO CRÍTICA (I2C) ---
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
            start = esp_timer_get_time(); // Início WCET
            mag.read(x, y, z);
            end = esp_timer_get_time();   // Fim WCET
        xSemaphoreGive(i2cMutex);
        // --- FIM I2C ---

        // Saída Serial fora do Mutex
        Serial.printf("MAG %d %d %d\n", x, y, z);
        digitalWrite(LED_MAG, LOW);

        uint32_t exec = end - start; // Tempo de execução (C_i)
        TMag.last_exec_us = exec;
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

    float pitch, roll, yaw;
    uint64_t start, end;

    while (true) {
        digitalWrite(LED_IMU, HIGH);

        // --- SEÇÃO CRÍTICA (I2C) ---
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
            start = esp_timer_get_time(); // Início WCET
            gyro.read();
            pitch = gyro.getPitch();
            roll  = gyro.getRoll();
            yaw   = gyro.getYaw();
            end = esp_timer_get_time();   // Fim WCET
        xSemaphoreGive(i2cMutex);
        // --- FIM I2C ---

        // Saída Serial fora do Mutex
        Serial.printf("IMU %.2f %.2f %.2f\n", pitch, roll, yaw);
        digitalWrite(LED_IMU, LOW);

        uint32_t exec = end - start; // Tempo de execução (C_i)
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
    
    float x, y, z;
    uint64_t start, end;

    while (true) {
        digitalWrite(LED_GYRO, HIGH);

        // --- SEÇÃO CRÍTICA (I2C) ---
        xSemaphoreTake(i2cMutex, portMAX_DELAY);
            start = esp_timer_get_time(); // Início WCET
            gyro.read();
            x = gyro.getAngleX();
            y = gyro.getAngleY();
            z = gyro.getAngleZ();
            end = esp_timer_get_time();   // Fim WCET
        xSemaphoreGive(i2cMutex);
        // --- FIM I2C ---

        // Saída Serial fora do Mutex
        Serial.printf("GYRO %.2f %.2f %.2f\n", x, y, z);
        digitalWrite(LED_GYRO, LOW);

        uint32_t exec = end - start; // Tempo de execução (C_i)
        TGyro.last_exec_us = exec;
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

    // Inicialização dos sensores - PONTO CRÍTICO
    // Se o QMC5883L não for encontrado, mag.begin() pode travar.
    mag.begin(Wire);
    
    // O WDT do SETUP pode expirar aqui se o sensor não responder.
    while (!gyro.wakeup()) {
        Serial.println("Falha IMU! Verifique conexões.");
        delay(1000);
    }
    gyro.setAccelSensitivity(2);
    gyro.setGyroSensitivity(1);

    i2cMutex = xSemaphoreCreateMutex();

    // AUMENTANDO O TAMANHO DA PILHA (STACK) PARA 8192 POR SEGURANÇA.
    xTaskCreatePinnedToCore(taskGyro, "GYRO", 8192, NULL, 3, NULL,  PRO_CPU_NUM); 
    xTaskCreatePinnedToCore(taskIMU,  "IMU",  8192, NULL, 2, NULL,PRO_CPU_NUM);
    xTaskCreatePinnedToCore(taskMag,  "MAG",  8192, NULL, 1, NULL,PRO_CPU_NUM);

    Serial.println("\nSistema iniciado. Tarefas rodando (WDT reset desativado).");
    
    // Remover delay e checkRM() para evitar WDT reset.
    // Se você quiser o checkRM(), adicione um vTaskDelay na loop() para agendar a execução.
}

void loop() {
    vTaskDelay(1);
}