#include <Arduino.h>

/* ===================== GPIOs ===================== */
#define LED_A      2
#define LED_B      4
#define LED_PLOT   5

/* ===================== PERÍODOS ===================== */
#define TA_PERIOD_US      20000   // 20 ms
#define TB_PERIOD_US      30000   // 30 ms
#define TPLOT_PERIOD_US   50000   // 50 ms

/* ===================== TEMPOS DE EXECUÇÃO ===================== */
#define TA_EXEC_US        2000    // 2 ms
#define TB_EXEC_US        3000    // 3 ms
#define TPLOT_EXEC_US     4000    // 4 ms
SemaphoreHandle_t i2cMutex;v
/* ===================== BUSY WAIT ===================== */
void busyWaitUs(uint32_t us)
{
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        taskYIELD(); // evita reset do watchdog
    }
}

/* ===================== Task A ===================== */
void taskA(void *p)
{
    TickType_t last = xTaskGetTickCount();
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    for(;;) {
        digitalWrite(LED_A, HIGH);
        busyWaitUs(TA_EXEC_US);
        digitalWrite(LED_A, LOW);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(TA_PERIOD_US/1000));
    }
}

/* ===================== Task B ===================== */
void taskB(void *p)
{
    TickType_t last = xTaskGetTickCount();

    for(;;) {
        digitalWrite(LED_B, HIGH);
        busyWaitUs(TB_EXEC_US);
        digitalWrite(LED_B, LOW);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(TB_PERIOD_US/1000));
    }
}

/* ===================== Task Plot ===================== */
void taskPlot(void *p)
{
    TickType_t last = xTaskGetTickCount();

    for(;;) {
        digitalWrite(LED_PLOT, HIGH);
        busyWaitUs(TPLOT_EXEC_US);
        digitalWrite(LED_PLOT, LOW);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(TPLOT_PERIOD_US/1000));
    }
}

/* ===================== Setup ===================== */
void setup()
{
    Serial.begin(115200);

    pinMode(LED_A, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_PLOT, OUTPUT);

    // PRIORIDADES RM (menor período = maior prioridade)
    xTaskCreatePinnedToCore(taskA, "A", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskB, "B", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskPlot, "Plot", 4096, NULL, 1, NULL, 1);
}

void loop() {}
