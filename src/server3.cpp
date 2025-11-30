// server.cpp - Código Final Ajustado com Medições (WCET/Jitter)
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

// ---------- VARIÁVEIS GLOBAIS DE MEDIÇÃO ----------
// WCET (Worst-Case Execution Time) - Em microssegundos (us)
volatile uint32_t wcet_mag = 0;
volatile uint32_t wcet_imu = 0;
volatile uint32_t wcet_gyro = 0;
volatile uint32_t wcet_server = 0;

// JITTER - Em microssegundos (us)
volatile uint32_t jitter_mag = 0;
volatile uint32_t jitter_imu = 0;
volatile uint32_t jitter_gyro = 0;

// Tempo da última ativação (para cálculo de Jitter)
volatile uint64_t last_mag_activation_us = 0;
volatile uint64_t last_imu_activation_us = 0;
volatile uint64_t last_gyro_activation_us = 0;

// ---------- SENSORES ----------
QMC5883L mag;
GY521 gyro(0x68);

// ---------- PINs / LEDs ----------
#define LED_MAG    2
#define LED_IMU    4 
#define LED_GYRO   5 
#define LED_SERVER 23

// ---------- RM periods (ms) ----------
#define MAG_PERIOD_MS   2000UL
#define IMU_PERIOD_MS   1000UL
#define GYRO_PERIOD_MS  500UL


// ---------- SERVER budget/period (us) ----------
#define SERVER_PERIOD_US  40000UL
#define SERVER_BUDGET_US   14000L // 14ms

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

// Defina um evento de notificação para a taskServer
TaskHandle_t xServerTaskHandle = NULL; 

// ---------- TASK MAG (periodic) ----------
void taskMag(void *p) {
  pinMode(LED_MAG, OUTPUT);
  // Variáveis necessárias para FreeRTOS e Leitura
  TickType_t lastWake = xTaskGetTickCount(); 
  sensor_packet_t pkt;
  int x, y, z; // Corrigido: Variáveis de leitura do sensor declaradas
  uint64_t current_time_us;
  uint32_t execution_time;
  
  // Inicializa o tempo de ativação para o jitter
  last_mag_activation_us = esp_timer_get_time();

  for (;;) {
    current_time_us = esp_timer_get_time();
    
    // --- 1. CÁLCULO DO JITTER ---
    uint64_t period_elapsed = current_time_us - last_mag_activation_us;
    last_mag_activation_us = current_time_us;
    
    // O Jitter é o desvio absoluto em relação ao período nominal (2000ms = 2000000 us)
    uint32_t deviation = abs((int32_t)period_elapsed - (int32_t)(MAG_PERIOD_MS * 1000UL));
    if (deviation > jitter_mag) {
      jitter_mag = deviation; // Atualiza o pior caso de Jitter
    }
    
    digitalWrite(LED_MAG, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint64_t t0 = esp_timer_get_time(); // Início da Execução
      mag.read(x, y, z);
      uint64_t t1 = esp_timer_get_time(); // Fim da Execução
      xSemaphoreGive(i2cMutex);

      // --- 2. CÁLCULO DO WCET (C) ---
      execution_time = (uint32_t)(t1 - t0);
      if (execution_time > wcet_mag) {
        wcet_mag = execution_time; // Atualiza o pior caso
      }

      // prepare packet
      strncpy(pkt.tag, "MAG", sizeof(pkt.tag));
      pkt.v1 = (float)x; pkt.v2 = (float)y; pkt.v3 = (float)z;
      pkt.ts_us = t1;

      // Envia para trás. Se cheia, descarta o item mais antigo e reenvia.
      if (xQueueSendToBack(sensorQueue, &pkt, 0) != pdTRUE) {
          sensor_packet_t dummyPkt;
          xQueueReceive(sensorQueue, &dummyPkt, 0); 
          xQueueSendToBack(sensorQueue, &pkt, 0);   
      }

      Serial.printf("MAG %d %d %d\n", x, y, z);
    } else {
      Serial.println("MAG: falha em pegar mutex I2C");
    }

    digitalWrite(LED_MAG, LOW);
    // Corrigido: lastWake está declarado no escopo da função
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MAG_PERIOD_MS));
  }
}

// ---------- TASK IMU (Pitch/Roll/Yaw) ----------
void taskIMU(void *p) {
  pinMode(LED_IMU, OUTPUT);
  // Variável necessária para FreeRTOS
  TickType_t lastWake = xTaskGetTickCount(); 
  sensor_packet_t pkt;
  float pitch, roll, yaw;
  
  uint64_t current_time_us;
  uint32_t execution_time;
  
  // Inicializa o tempo de ativação para o jitter
  last_imu_activation_us = esp_timer_get_time();

  for (;;) {
    current_time_us = esp_timer_get_time();
    
    // --- 1. CÁLCULO DO JITTER ---
    uint64_t period_elapsed = current_time_us - last_imu_activation_us;
    last_imu_activation_us = current_time_us;
    
    // O Período nominal é IMU_PERIOD_MS (1000ms) = 1000000 us
    uint32_t deviation = abs((int32_t)period_elapsed - (int32_t)(IMU_PERIOD_MS * 1000UL));
    if (deviation > jitter_imu) {
      jitter_imu = deviation; // Atualiza o pior caso de Jitter
    }

    digitalWrite(LED_IMU, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint64_t t0 = esp_timer_get_time(); // Início da Execução (C)
      
      gyro.read();
      pitch = gyro.getPitch();
      roll  = gyro.getRoll();
      yaw   = gyro.getYaw();
      
      uint64_t t1 = esp_timer_get_time(); // Fim da Execução (C)
      xSemaphoreGive(i2cMutex);
      
      // --- 2. CÁLCULO DO WCET (C) ---
      execution_time = (uint32_t)(t1 - t0);
      if (execution_time > wcet_imu) {
        wcet_imu = execution_time; // Atualiza o pior caso
      }

      strncpy(pkt.tag, "IMU", sizeof(pkt.tag));
      pkt.v1 = pitch; pkt.v2 = roll; pkt.v3 = yaw;
      pkt.ts_us = t1;

      // Envia para trás. Se cheia, descarta o item mais antigo e reenvia.
      if (xQueueSendToBack(sensorQueue, &pkt, 0) != pdTRUE) {
          sensor_packet_t dummyPkt;
          xQueueReceive(sensorQueue, &dummyPkt, 0); 
          xQueueSendToBack(sensorQueue, &pkt, 0);   
      }

      Serial.printf("IMU %.2f %.2f %.2f\n", pitch, roll, yaw);
    } else {
      Serial.println("IMU: falha em pegar mutex I2C");
    }

    digitalWrite(LED_IMU, LOW);
    // Corrigido: lastWake está declarado no escopo da função
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));
  }
}

// ---------- TASK GYRO (angleX/Y/Z) ----------
void taskGyro(void *p) {
  pinMode(LED_GYRO, OUTPUT);
  // Variável necessária para FreeRTOS
  TickType_t lastWake = xTaskGetTickCount(); 
  sensor_packet_t pkt;
  float ax, ay, az;
  
  uint64_t current_time_us;
  uint32_t execution_time;
  
  // Inicializa o tempo de ativação para o jitter
  last_gyro_activation_us = esp_timer_get_time();

  for (;;) {
    current_time_us = esp_timer_get_time();
    
    // --- 1. CÁLCULO DO JITTER ---
    uint64_t period_elapsed = current_time_us - last_gyro_activation_us;
    last_gyro_activation_us = current_time_us;
    
    // O Período nominal é GYRO_PERIOD_MS (500ms) = 500000 us
    uint32_t deviation = abs((int32_t)period_elapsed - (int32_t)(GYRO_PERIOD_MS * 1000UL));
    if (deviation > jitter_gyro) {
      jitter_gyro = deviation; // Atualiza o pior caso de Jitter
    }

    digitalWrite(LED_GYRO, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint64_t t0 = esp_timer_get_time(); // Início da Execução (C)
      
      gyro.read();
      ax = gyro.getAngleX();
      ay = gyro.getAngleY();
      az = gyro.getAngleZ();
      
      uint64_t t1 = esp_timer_get_time(); // Fim da Execução (C)
      xSemaphoreGive(i2cMutex);

      // --- 2. CÁLCULO DO WCET (C) ---
      execution_time = (uint32_t)(t1 - t0);
      if (execution_time > wcet_gyro) {
        wcet_gyro = execution_time; // Atualiza o pior caso
      }

      strncpy(pkt.tag, "GYRO", sizeof(pkt.tag));
      pkt.v1 = ax; pkt.v2 = ay; pkt.v3 = az;
      pkt.ts_us = t1;

      // Envia para trás. Se cheia, descarta o item mais antigo e reenvia.
      if (xQueueSendToBack(sensorQueue, &pkt, 0) != pdTRUE) {
          sensor_packet_t dummyPkt;
          xQueueReceive(sensorQueue, &dummyPkt, 0); 
          xQueueSendToBack(sensorQueue, &pkt, 0);   
      }

      Serial.printf("GYRO %.2f %.2f %.2f\n", ax, ay, az);
    } else {
      Serial.println("GYRO: falha em pegar mutex I2C");
    }

    digitalWrite(LED_GYRO, LOW);
    // Corrigido: lastWake está declarado no escopo da função
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GYRO_PERIOD_MS));
  }
}

void taskServer(void *p) {
    xServerTaskHandle = xTaskGetCurrentTaskHandle();
    // Variável necessária para FreeRTOS
    TickType_t lastWake = xTaskGetTickCount(); 
    uint32_t execution_time;
    
    for (;;) {
        // Recarrega o orçamento no início de cada período
        server_budget_us = SERVER_BUDGET_US;
        
        // Espera por um evento (GET) ou até o final do período do Servidor (40ms)
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SERVER_PERIOD_US/1000)) > 0) {
            
            // O Servidor foi ACIONADO (interrompendo as periódicas)
            if (request_pending && server_budget_us > 0) {
                
                digitalWrite(LED_SERVER, HIGH); 
                
                uint64_t t0 = esp_timer_get_time();

                // ----------------------------------------------------
                // ⚡ LÓGICA DE PROCESSAMENTO DO PEDIDO (Gasta o Budget)
                // ----------------------------------------------------
                
                // Coleta e JSON (Drena a fila e monta o JSON)
                sensor_packet_t pkt;
                String json = "";
                while(xQueueReceive(sensorQueue, &pkt, 0) == pdTRUE) {
                   json += String(pkt.tag) + ":" + String(pkt.v1) + "," + String(pkt.v2) + "," + String(pkt.v3) + "|" ;
                }

                // Send to client
                if (client && client.connected()) {
                    client.print(json);
                    client.print('\n');
                    client.flush();
                }
                // ----------------------------------------------------

                uint64_t t1 = esp_timer_get_time();
                int32_t used = (int32_t)(t1 - t0);
                
                // --- 1. CÁLCULO DO WCET (C) ---
                execution_time = (uint32_t)used;
                if (execution_time > wcet_server) {
                  wcet_server = execution_time; // Atualiza o pior caso
                }

                // Dedução do Orçamento
                server_budget_us -= used;
                // Serial.printf("[SERVER] Usado: %ld us. Orçamento Restante: %ld us\n", used, server_budget_us);
                
                if (server_budget_us < 0) {
                   server_budget_us = 0; 
                   // Serial.println("WARNING: Server estourou o budget!");
                }
                request_pending = false;

                digitalWrite(LED_SERVER, LOW); 
            }
        } 
        
        // Espera o próximo período para recarregar o budget.
        // Corrigido: lastWake está declarado no escopo da função
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SERVER_PERIOD_US/1000));
    }
}

void taskListener(void *p) {
  Serial.println("[LISTEN] Entrando em loop de aceitação do cliente.");
  
  for (;;) {
    if (!client) {
      client = tcpServer.available(); 
      if (client) {
        Serial.println("[LISTEN] Cliente conectado");
      }
    } else {
      if (!client.connected()) {
        client.stop();
        Serial.println("[LISTEN] Cliente desconectado");
        client = WiFiClient();
      } else if (client.available()) {
        String line = client.readStringUntil('\n');
        line.trim();
        if (line == "GET") {
                    if (xServerTaskHandle != NULL) {
                        xTaskNotifyGive(xServerTaskHandle); // Aciona o servidor
                    }
                   
                    request_pending = true; 
                    Serial.println("[LISTEN] GET recebido -> Server Notificado!");
                }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- SETUP ----------

void setup() {
  disableCore0WDT();
  disableCore1WDT();

  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Inicializando...");

  Wire.begin(21, 22);

  // create IPC objects
  i2cMutex = xSemaphoreCreateMutex();
  sensorQueue = xQueueCreate(8, sizeof(sensor_packet_t)); 

  // init sensors (I2C)
  if (!mag.begin(Wire)) { Serial.println("ERRO: mag.begin() falhou"); }
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

  // Inicializa o Servidor TCP
  tcpServer.begin(); 
  Serial.println("TCP server started on port 5005 (in setup).");

  // Create tasks (RM: smaller period -> higher priority)
  // Tarefas de Sensor (Prioridades 1, 2, 3) - PRO_CPU
  xTaskCreatePinnedToCore(taskGyro, "GYRO", 6144, NULL, 3, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskIMU,  "IMU",  6144, NULL, 2, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskMag,  "MAG",  6144, NULL, 1, NULL, PRO_CPU_NUM);
  
  // Tarefas de Rede (Prioridades) - PRO_CPU
  xTaskCreatePinnedToCore(taskServer,   "Server", 4096, NULL, 4, &xServerTaskHandle, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskListener, "Listen", 4096, NULL, 5, NULL, PRO_CPU_NUM);

  Serial.println("Sistema pronto.");
}

void loop() {
  // Relatório de Métricas no loop()
  Serial.println("\n--- MÉTRICAS DE TEMPO REAL ---");

  // Relatório de WCET (C)
  Serial.printf("WCET MAG: %u us\n", wcet_mag);
  Serial.printf("WCET IMU: %u us\n", wcet_imu);
  Serial.printf("WCET GYRO: %u us\n", wcet_gyro);
  Serial.printf("WCET SERVER (C_s): %u us (Budget: 14000 us)\n", wcet_server);
  
  // Relatório de JITTER (Variação de período)
  Serial.printf("JITTER MAG (max desvio de 2000ms): %u us\n", jitter_mag);
  Serial.printf("JITTER IMU (max desvio de 1000ms): %u us\n", jitter_imu);
  Serial.printf("JITTER GYRO (max desvio de 500ms): %u us\n", jitter_gyro);

  vTaskDelay(pdMS_TO_TICKS(5000)); // Relata a cada 5 segundos
}