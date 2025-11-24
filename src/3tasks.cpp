#include <Arduino.h>
#include <WiFi.h>

/* ===================== CONFIG WiFi AP ===================== */
const char *AP_SSID = "ESP32_AP_RM";
const char *AP_PASS = "12345678";

WiFiServer tcpServer(5005);
WiFiClient client;

/* ===================== GPIOs ===================== */
#define LED_A   2
#define LED_B   4
#define LED_PLOT 5
#define LED_SERVER 18

/* ===================== PERÍODOS ===================== */
#define TA_PERIOD_US      20000
#define TB_PERIOD_US      30000
#define TPLOT_PERIOD_US   50000

#define TA_EXEC_US        2000
#define TB_EXEC_US        3000
#define TPLOT_EXEC_US     4000

/* ===================== SERVIDOR DEFERRÁVEL ===================== */
#define SERVER_PERIOD_US      40000
#define SERVER_BUDGET_US      7000

volatile int32_t server_budget_us = 0;
volatile bool request_pending = false;

/* ===================== BUSY WAIT SEM RESET ===================== */
void busyWaitUs(uint32_t us) {
    uint64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        // yield evita reset de watchdog
        taskYIELD();
    }
}

/* ===================== Task A ===================== */
void taskA(void *p) {
    TickType_t last = xTaskGetTickCount();

    for(;;) {
        digitalWrite(LED_A, HIGH);
        busyWaitUs(TA_EXEC_US);
        digitalWrite(LED_A, LOW);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(TA_PERIOD_US/1000));
    }
}

/* ===================== Task B ===================== */
void taskB(void *p) {
    TickType_t last = xTaskGetTickCount();

    for(;;) {
        digitalWrite(LED_B, HIGH);
        busyWaitUs(TB_EXEC_US);
        digitalWrite(LED_B, LOW);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(TB_PERIOD_US/1000));
    }
}

/* ===================== Task plot ===================== */
void taskPlot(void *p) {
    TickType_t last = xTaskGetTickCount();

    for(;;) {
        digitalWrite(LED_PLOT, HIGH);
        busyWaitUs(TPLOT_EXEC_US);
        digitalWrite(LED_PLOT, LOW);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(TPLOT_PERIOD_US/1000));
    }
}

/* ===================== Aperiodic Server ===================== */
void taskServer(void *p)
{
    TickType_t last = xTaskGetTickCount();

    for(;;) {
        server_budget_us = SERVER_BUDGET_US;
        uint64_t start_period = esp_timer_get_time();

        while ((esp_timer_get_time() - start_period) < SERVER_PERIOD_US) 
        {
            if (request_pending && server_budget_us > 0) {
                digitalWrite(LED_SERVER, HIGH);

                uint64_t t0 = esp_timer_get_time();

                if (client && client.connected()) {
                    client.println("SENSORES_OK");
                }

                busyWaitUs(2000); // custo

                uint64_t t1 = esp_timer_get_time();
                server_budget_us -= (t1 - t0);
                request_pending = false;

                digitalWrite(LED_SERVER, LOW);
            }

            taskYIELD();
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(SERVER_PERIOD_US/1000));
    }
}

/* ===================== Listener TCP ===================== */
void taskListener(void *p)
{
    tcpServer.begin();

    for(;;)
    {
        if (!client) {
            client = tcpServer.available();
        }
        else if (client.available()) {
            String cmd = client.readStringUntil('\n');
            cmd.trim();
            if (cmd == "GET") {
                request_pending = true;
            }
        }

        vTaskDelay(1);  // obrigatório para não resetar
    }
}

/* ===================== Setup ===================== */
void setup() {
    Serial.begin(115200);

    pinMode(LED_A, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(LED_PLOT, OUTPUT);
    pinMode(LED_SERVER, OUTPUT);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    Serial.print("AP iniciado em: ");
    Serial.println(WiFi.softAPIP());

    // Prioridades RM
    xTaskCreatePinnedToCore(taskA, "A", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(taskB, "B", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskPlot, "Plot", 4096, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(taskServer, "Serv", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(taskListener, "Listen", 4096, NULL, 5, NULL, 0);
}

void loop() {}
