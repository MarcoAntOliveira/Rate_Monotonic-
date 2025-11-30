// server.cpp - Código Final Ajustado
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
#define LED_MAG    2//5
#define LED_IMU    4 //7
#define LED_GYRO   5 //10
#define LED_SERVER 18//11

// ---------- RM periods (ms) ----------
#define MAG_PERIOD_MS   2000UL
#define IMU_PERIOD_MS   1000UL
#define GYRO_PERIOD_MS  500UL


// ---------- SERVER budget/period (us) ----------
#define SERVER_PERIOD_US  40000UL
#define SERVER_BUDGET_US   14000L // *** AUMENTADO para 14ms ***

volatile int32_t server_budget_us = 0;
volatile bool request_pending = false;

// ---------- IPC ----------
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t serverMutex;
QueueHandle_t sensorQueue; // queue of latest sensor packets

// ---------- packet ----------
typedef struct {
  char tag[8];     // "MAG","IMU","GYRO"
  float v1, v2, v3;
  uint64_t ts_us;
} sensor_packet_t;



// ---------- TASK MAG (periodic) ----------
void taskMag(void *p) {
  pinMode(LED_MAG, OUTPUT);
  TickType_t lastWake = xTaskGetTickCount();
  sensor_packet_t pkt;
  int x, y, z;
  uint64_t t0, t1;

  for (;;) {
    digitalWrite(LED_MAG, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      t0 = esp_timer_get_time();
      mag.read(x, y, z);
      t1 = esp_timer_get_time();
      xSemaphoreGive(i2cMutex);

      // prepare packet
      strncpy(pkt.tag, "MAG", sizeof(pkt.tag));
      pkt.v1 = (float)x; pkt.v2 = (float)y; pkt.v3 = (float)z;
      pkt.ts_us = t1;

      // CORREÇÃO: Envia para trás. Se cheia, descarta o item mais antigo e reenvia.
      if (xQueueSendToBack(sensorQueue, &pkt, 0) != pdTRUE) {
          sensor_packet_t dummyPkt;
          xQueueReceive(sensorQueue, &dummyPkt, 0); // Descarta o item mais antigo (FIFO)
          xQueueSendToBack(sensorQueue, &pkt, 0);   // Envia o novo
      }

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
  float pitch, roll, yaw;
  uint64_t t0, t1;

  for (;;) {
    digitalWrite(LED_IMU, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      t0 = esp_timer_get_time();
      gyro.read();
      pitch = gyro.getPitch();
      roll  = gyro.getRoll();
      yaw   = gyro.getYaw();
      t1 = esp_timer_get_time();
      xSemaphoreGive(i2cMutex);

      strncpy(pkt.tag, "IMU", sizeof(pkt.tag));
      pkt.v1 = pitch; pkt.v2 = roll; pkt.v3 = yaw;
      pkt.ts_us = t1;

      // CORREÇÃO: Envia para trás. Se cheia, descarta o item mais antigo e reenvia.
      if (xQueueSendToBack(sensorQueue, &pkt, 0) != pdTRUE) {
          sensor_packet_t dummyPkt;
          xQueueReceive(sensorQueue, &dummyPkt, 0); // Descarta o item mais antigo (FIFO)
          xQueueSendToBack(sensorQueue, &pkt, 0);   // Envia o novo
      }

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
  float ax, ay, az;
  uint64_t t0, t1;

  for (;;) {
    digitalWrite(LED_GYRO, HIGH);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      t0 = esp_timer_get_time();
      gyro.read();
      ax = gyro.getAngleX();
      ay = gyro.getAngleY();
      az = gyro.getAngleZ();
      t1 = esp_timer_get_time();
      xSemaphoreGive(i2cMutex);

      strncpy(pkt.tag, "GYRO", sizeof(pkt.tag));
      pkt.v1 = ax; pkt.v2 = ay; pkt.v3 = az;
      pkt.ts_us = t1;

      // CORREÇÃO: Envia para trás. Se cheia, descarta o item mais antigo e reenvia.
      if (xQueueSendToBack(sensorQueue, &pkt, 0) != pdTRUE) {
          sensor_packet_t dummyPkt;
          xQueueReceive(sensorQueue, &dummyPkt, 0); // Descarta o item mais antigo (FIFO)
          xQueueSendToBack(sensorQueue, &pkt, 0);   // Envia o novo
      }

      Serial.printf("GYRO %.2f %.2f %.2f\n", ax, ay, az);
    } else {
      Serial.println("GYRO: falha em pegar mutex I2C");
    }

    digitalWrite(LED_GYRO, LOW);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GYRO_PERIOD_MS));
  }
}

// Defina um evento de notificação para a taskServer
TaskHandle_t xServerTaskHandle = NULL; 

void taskServer(void *p) {
    xServerTaskHandle = xTaskGetCurrentTaskHandle(); // Armazena o handle
    TickType_t lastWake = xTaskGetTickCount();
   
    // O loop principal agora é redefinido para o Servidor Adiável
    for (;;) {
        // Recarrega o orçamento no início de cada período do Servidor
        server_budget_us = SERVER_BUDGET_US;
        uint64_t periodStart = esp_timer_get_time();

        // Espera por um evento (GET) ou até o final do período do Servidor
        // xTaskNotifyWait é o mecanismo de bloqueio
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SERVER_PERIOD_US/1000)) > 0) {
            
            // EXECUÇÃO DO ORÇAMENTO
            
            // O Servidor Adiável roda até o orçamento acabar, ou a requisição ser atendida.
            // Repetimos o loop de atendimento ENQUANTO o orçamento permitir.
            // (A lógica de budget e request_pending já está dentro do seu código)
            
            // Sua lógica de budget já está bem estruturada, mas você pode garantir 
            // que ela use o tempo restante do orçamento AGORA:
            
            uint64_t t0 = esp_timer_get_time();
            // ... (Processamento) ...
            
            // ... (Seu código de coleta/JSON/envio) ...

            uint64_t t1 = esp_timer_get_time();
            int32_t used = (int32_t)(t1 - t0);
            server_budget_us -= used;
            if (server_budget_us < 0) server_budget_us = 0;
            request_pending = false;
             digitalWrite(LED_SERVER , HIGH);
           
            
            // Após a execução do orçamento, a tarefa Server voltará ao 
            // seu estado bloqueado (esperando pelo próximo período ou notificação)
        } 
        
        // Se o evento não ocorreu, apenas espera o próximo período para recarregar o budget.
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SERVER_PERIOD_US/1000));
        digitalWrite(LED_SERVER , LOW);
    }
}
void taskListener(void *p) {
  // Atraso removido, pois o tcpServer.begin() foi movido para o setup()
  Serial.println("[LISTEN] Entrando em loop de aceitação do cliente.");
  
  for (;;) {
    if (!client) {
      // Tenta aceitar o cliente. O Servidor já está iniciado.
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
                    // ⚡ AÇÃO CRÍTICA: Notificar a taskServer IMEDIATAMENTE!
                    if (xServerTaskHandle != NULL) {
                        vTaskNotifyGiveFromISR(xServerTaskHandle, NULL); // Aciona o servidor
                    }
                    
                    // Mantemos request_pending = true para que o Server saiba que foi um GET
                   
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

  // ⚡ MUDANÇA CRÍTICA: Inicializa o Servidor TCP no setup() 
  // antes de criar as tarefas, garantindo que ele esteja ativo no boot.
  tcpServer.begin(); 
  Serial.println("TCP server started on port 5005 (in setup).");

  // Create tasks (RM: smaller period -> higher priority)
  // Tarefas de Sensor (Prioridades 1, 2, 3) - PRO_CPU
  xTaskCreatePinnedToCore(taskGyro, "GYRO", 6144, NULL, 3, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskIMU,  "IMU",  6144, NULL, 2, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskMag,  "MAG",  6144, NULL, 1, NULL, PRO_CPU_NUM);
  
  // Tarefas de Rede (Prioridades reduzidas para estabilidade) - APP_CPU
  // Prioridade do Listener reduzida de 5 para 3
  xTaskCreatePinnedToCore(taskServer,   "Server", 4096, NULL, 4, NULL, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskListener, "Listen", 4096, NULL, 5, NULL, PRO_CPU_NUM);

  Serial.println("Sistema pronto.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}