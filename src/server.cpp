// server.cpp
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "GY521.h"
#include "QMC5883L.h"

// ---------- CONFIG WiFi AP ----------
const char *AP_SSID = "ESP32_AP_RM";
const char *AP_PASS = "12345678";
WiFiServer tcpServer(5005);
WiFiClient client;

// ---------- SENSORES ----------
QMC5883L mag;
GY521 gyro(0x68);

// ---------- PINs / LEDs ----------
#define LED_MAG    2
#define LED_IMU    4
#define LED_GYRO   5
#define LED_SERVER 18

// ---------- RM periods (ms) ----------
#define MAG_PERIOD_MS   2000UL
#define IMU_PERIOD_MS   1000UL
#define GYRO_PERIOD_MS  500UL

// ---------- SERVER budget/period (us) ----------
#define SERVER_PERIOD_US  40000UL
#define SERVER_BUDGET_US   7000L

volatile int32_t server_budget_us = 0;
volatile bool request_pending = false;

// ---------- IPC ----------
SemaphoreHandle_t i2cMutex;
QueueHandle_t sensorQueue; // queue of latest sensor packets

// ---------- packet ----------
typedef struct {
  char tag[8];     // "MAG","IMU","GYRO"
  float v1, v2, v3;
  uint64_t ts_us;
} sensor_packet_t;

// ---------- small busy wait (use delayMicroseconds inside task safe) ----------
static inline void busyWaitUs(uint32_t us) {
  uint64_t start = esp_timer_get_time();
  while ((esp_timer_get_time() - start) < us) {
    taskYIELD();
  }
}

// ---------- TASK MAG (periodic) ----------
void taskMag(void *p) {
  pinMode(LED_MAG, OUTPUT);
  TickType_t lastWake = xTaskGetTickCount();
  sensor_packet_t pkt;

  for (;;) {
    digitalWrite(LED_MAG, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint64_t t0 = esp_timer_get_time();
      int x, y, z;
      mag.read(x, y, z);
      uint64_t t1 = esp_timer_get_time();
      xSemaphoreGive(i2cMutex);

      // prepare packet
      strncpy(pkt.tag, "MAG", sizeof(pkt.tag));
      pkt.v1 = (float)x; pkt.v2 = (float)y; pkt.v3 = (float)z;
      pkt.ts_us = t1;

      // send to queue (latest values). If full, overwrite oldest.
      if (xQueueSend(sensorQueue, &pkt, 0) != pdTRUE) {
        // try overwrite
        xQueueOverwrite(sensorQueue, &pkt);
      }

      // also print (for debug)
      Serial.printf("MAG %d %d %d\n", x, y, z);
    } else {
      Serial.println("MAG: falha em pegar mutex I2C");
    }

    digitalWrite(LED_MAG, LOW);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MAG_PERIOD_MS));
  }
}

// ---------- TASK IMU (Pitch/Roll/Yaw) ----------
void taskIMU(void *p) {
  pinMode(LED_IMU, OUTPUT);
  TickType_t lastWake = xTaskGetTickCount();
  sensor_packet_t pkt;

  for (;;) {
    digitalWrite(LED_IMU, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint64_t t0 = esp_timer_get_time();
      gyro.read();
      float pitch = gyro.getPitch();
      float roll  = gyro.getRoll();
      float yaw   = gyro.getYaw();
      uint64_t t1 = esp_timer_get_time();
      xSemaphoreGive(i2cMutex);

      strncpy(pkt.tag, "IMU", sizeof(pkt.tag));
      pkt.v1 = pitch; pkt.v2 = roll; pkt.v3 = yaw;
      pkt.ts_us = t1;

      if (xQueueSend(sensorQueue, &pkt, 0) != pdTRUE) xQueueOverwrite(sensorQueue, &pkt);

      Serial.printf("IMU %.2f %.2f %.2f\n", pitch, roll, yaw);
    } else {
      Serial.println("IMU: falha em pegar mutex I2C");
    }

    digitalWrite(LED_IMU, LOW);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));
  }
}

// ---------- TASK GYRO (angleX/Y/Z) ----------
void taskGyro(void *p) {
  pinMode(LED_GYRO, OUTPUT);
  TickType_t lastWake = xTaskGetTickCount();
  sensor_packet_t pkt;

  for (;;) {
    digitalWrite(LED_GYRO, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint64_t t0 = esp_timer_get_time();
      gyro.read();
      float ax = gyro.getAngleX();
      float ay = gyro.getAngleY();
      float az = gyro.getAngleZ();
      uint64_t t1 = esp_timer_get_time();
      xSemaphoreGive(i2cMutex);

      strncpy(pkt.tag, "GYRO", sizeof(pkt.tag));
      pkt.v1 = ax; pkt.v2 = ay; pkt.v3 = az;
      pkt.ts_us = t1;

      if (xQueueSend(sensorQueue, &pkt, 0) != pdTRUE) xQueueOverwrite(sensorQueue, &pkt);

      Serial.printf("GYRO %.2f %.2f %.2f\n", ax, ay, az);
    } else {
      Serial.println("GYRO: falha em pegar mutex I2C");
    }

    digitalWrite(LED_GYRO, LOW);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GYRO_PERIOD_MS));
  }
}

// ---------- SERVER DEFERRABLE (budgeted) ----------
void taskServer(void *p) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    server_budget_us = SERVER_BUDGET_US;
    uint64_t periodStart = esp_timer_get_time();

    // during period, respond to pending request as long as budget > 0
    while ((esp_timer_get_time() - periodStart) < SERVER_PERIOD_US) {
      if (request_pending && server_budget_us > 0) {
        digitalWrite(LED_SERVER, HIGH);
        uint64_t t0 = esp_timer_get_time();

        // Collect latest packets from queue (drain)
        sensor_packet_t pkt;
        // We will keep last seen values per tag
        bool haveMAG=false, haveIMU=false, haveGYRO=false;
        sensor_packet_t lastMAG, lastIMU, lastGYRO;

        // drain queue (non-blocking)
        while (xQueueReceive(sensorQueue, &pkt, 0) == pdTRUE) {
          if (strncmp(pkt.tag, "MAG", 3)==0) { lastMAG = pkt; haveMAG=true; }
          else if (strncmp(pkt.tag, "IMU", 3)==0) { lastIMU = pkt; haveIMU=true; }
          else if (strncmp(pkt.tag, "GYRO",4)==0) { lastGYRO = pkt; haveGYRO=true; }
        }

        // Build JSON string (simple manual)
        String json = "{";
        if (haveMAG) {
          json += "\"MAG\":[" + String((int)lastMAG.v1) + "," + String((int)lastMAG.v2) + "," + String((int)lastMAG.v3) + "],";
        } else json += "\"MAG\":null,";

        if (haveIMU) {
          json += "\"IMU\":[" + String(lastIMU.v1,2) + "," + String(lastIMU.v2,2) + "," + String(lastIMU.v3,2) + "],";
        } else json += "\"IMU\":null,";

        if (haveGYRO) {
          json += "\"GYRO\":[" + String(lastGYRO.v1,2) + "," + String(lastGYRO.v2,2) + "," + String(lastGYRO.v3,2) + "],";
        } else json += "\"GYRO\":null,";

        json += "\"ts\":" + String(esp_timer_get_time());
        json += "}";

        // Send to client if connected
        if (client && client.connected()) {
          client.println(json);
        }

        // simulate small processing cost (and account to budget)
        busyWaitUs(2000);

        uint64_t t1 = esp_timer_get_time();
        int32_t used = (int32_t)(t1 - t0);
        server_budget_us -= used;
        if (server_budget_us < 0) server_budget_us = 0;
        request_pending = false;

        digitalWrite(LED_SERVER, LOW);
      }
      taskYIELD();
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SERVER_PERIOD_US/1000));
  }
}

// ---------- LISTENER (accept + parse GET) ----------
void taskListener(void *p) {
  tcpServer.begin();
  Serial.println("TCP server started on port 5005");

  for (;;) {
    if (!client) {
      client = tcpServer.available();
      if (client) {
        Serial.println("Cliente conectado");
      }
    } else {
      if (!client.connected()) {
        client.stop();
        Serial.println("Cliente desconectado");
        // set to null so next loop accepts new client
        client = WiFiClient();
      } else if (client.available()) {
        String line = client.readStringUntil('\n');
        line.trim();
        if (line == "GET") {
          request_pending = true;
          Serial.println("GET recebido -> request_pending = true");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- SETUP ----------

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Inicializando...");

  Wire.begin(21, 22);

  // create IPC objects
  i2cMutex = xSemaphoreCreateMutex();
  sensorQueue = xQueueCreate(8, sizeof(sensor_packet_t)); // small ring buffer

  // init sensors (non-blocking style)
  if (!mag.begin(Wire)) {
    Serial.println("ERRO: mag.begin() falhou");
    // continue anyway
  }

  // gyro wakeup loop with small delay to avoid WDT
  uint32_t tries=0;
  while (!gyro.wakeup() && tries < 10) {
    Serial.println("Tentando wakeup GY521...");
    vTaskDelay(pdMS_TO_TICKS(200));
    tries++;
  }
  if (tries==10) Serial.println("WARNING: GY521 wakeup falhou");

  gyro.setAccelSensitivity(2);
  gyro.setGyroSensitivity(1);
  gyro.setThrottle();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Create tasks (RM: smaller period -> higher priority)
  xTaskCreatePinnedToCore(taskGyro, "GYRO", 6144, NULL, 3, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskIMU,  "IMU",  6144, NULL, 2, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskMag,  "MAG",  6144, NULL, 1, NULL, PRO_CPU_NUM);

  xTaskCreatePinnedToCore(taskServer,   "Server", 4096, NULL, 4, NULL, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskListener, "Listen", 4096, NULL, 5, NULL, APP_CPU_NUM);

  Serial.println("Sistema pronto.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
