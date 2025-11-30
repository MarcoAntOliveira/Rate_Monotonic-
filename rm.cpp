#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// Bibliotecas Adafruit (HMC5883) ainda necessárias para o magnetômetro
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// --- CONFIGURAÇÃO DE HARDWARE E ENDEREÇOS ---
const int MPU_ADDR = 0x68; // Endereço I2C do MPU-6050
#define PIN_LED_MAG      2
#define PIN_LED_MPU      4
#define PIN_LED_FILTRO   5
#define PIN_LED_DEADLINE 15 // Novo pino para indicar perda de deadline

// --- CONFIGURAÇÃO RTOS E TEMPOS ---
#define PERIOD_MAG_MS       40    
#define PERIOD_MPU_MS       30    // MPU6050 Acelerômetro/Giroscópio
#define PERIOD_FILTRO_MS    60

// Tempos de simulação (em microsegundos)
#define computacao_mag      25000   
#define computacao_mpu      10000  
#define computacao_filtro   12000 

// --- ESTRUTURAS DE DADOS E SINCRONIZAÇÃO ---
// Estrutura para dados compartilhados entre tarefas
typedef struct {
  // MPU6050 (Acelerômetro/Giroscópio)
  int16_t accel_x, accel_y, accel_z;
  int16_t gyro_x, gyro_y, gyro_z;
  uint64_t lastMpuTimeUs;
  
  // HMC5883 (Magnetômetro)
  int32_t mag_x, mag_y, mag_z;
  uint64_t lastMagTimeUs;

  // Saída do filtro
  long avg_accel_x, avg_accel_y, avg_accel_z;
  long avg_gyro_x, avg_gyro_y, avg_gyro_z;

} SharedData_t;

SharedData_t sharedData;

// Mutexes
SemaphoreHandle_t i2cMutex; // Mutex para proteger o barramento I2C (Wire.h)
SemaphoreHandle_t dataMutex; // Mutex para proteger a estrutura sharedData

// --- VARIÁVEIS GLOBAIS DE SENSORES ---
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// --- FUNÇÕES DE UTENSÍLIOS ---

// Simula tempo de processamento consumindo CPU
void busyWaitMicroseconds(uint32_t us) {
  uint66_t start = esp_timer_get_time();
  while ((esp_timer_get_time() - start) < us) {
    asm volatile("nop"); // No Operation
  }
}

// Inicializa e acorda o MPU6050
void mpu6050_wake_up() {
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

// --- TAREFAS FREE RTOS ---

// Tarefa MPU6050: Acelerômetro e Giroscópio
void taskacelerometro(void* param){
  const TickType_t periodTicks = pdMS_TO_TICKS(PERIOD_MPU_MS);
  TickType_t ativacao = xTaskGetTickCount();
  
  int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature;
  
  // Inicialização específica da tarefa MPU
  xSemaphoreTake(i2cMutex, portMAX_DELAY);
  mpu6050_wake_up();
  xSemaphoreGive(i2cMutex);

  while(true){
    uint64_t t0 = esp_timer_get_time();
    
    // Indica início da execução
    digitalWrite(PIN_LED_MPU, HIGH);

    // --- LEITURA MPU6050 (I2C Bare-Metal) ---
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B); // Começa em ACCEL_XOUT_H
      Wire.endTransmission(false); // Mantém conexão ativa para leitura
      // 7*2 = 14 bytes (3x Accel, 1x Temp, 3x Gyro)
      if (Wire.requestFrom(MPU_ADDR, 7*2, true) == 14) { 
        // Lendo Accel (MSB | LSB)
        accelerometer_x = Wire.read()<<8 | Wire.read(); 
        accelerometer_y = Wire.read()<<8 | Wire.read(); 
        accelerometer_z = Wire.read()<<8 | Wire.read(); 
        // Lendo Temperatura (Ignorada, mas lida para avançar o ponteiro)
        temperature     = Wire.read()<<8 | Wire.read(); 
        // Lendo Gyro (MSB | LSB)
        gyro_x          = Wire.read()<<8 | Wire.read(); 
        gyro_y          = Wire.read()<<8 | Wire.read(); 
        gyro_z          = Wire.read()<<8 | Wire.read(); 
      } else {
        Serial.println("[MPU] ERRO: Falha ao ler 14 bytes do MPU6050.");
        accelerometer_x = accelerometer_y = accelerometer_z = 0;
        gyro_x = gyro_y = gyro_z = 0;
      }
      
    xSemaphoreGive(i2cMutex);
    
    // Simulação de CPU Load para leitura
    busyWaitMicroseconds(computacao_mpu);
    
    // Atualiza dados compartilhados
    xSemaphoreTake(dataMutex, portMAX_DELAY);
      sharedData.accel_x = accelerometer_x;
      sharedData.accel_y = accelerometer_y;
      sharedData.accel_z = accelerometer_z;
      sharedData.gyro_x = gyro_x;
      sharedData.gyro_y = gyro_y;
      sharedData.gyro_z = gyro_z;
      sharedData.lastMpuTimeUs = esp_timer_get_time();
    xSemaphoreGive(dataMutex);

    // Indica fim da execução
    digitalWrite(PIN_LED_MPU, LOW);

    uint64_t t1 = esp_timer_get_time();
    uint64_t execucao = t1 - t0;

    // Checa se o deadline foi perdido
    if (execucao > PERIOD_MPU_MS * 1000) {
      Serial.printf("[MPU] Deadline PERDIDO! Exec=%lluus, Period=%dms\n", execucao, PERIOD_MPU_MS);
      digitalWrite(PIN_LED_DEADLINE, HIGH);
      delay(1); 
      digitalWrite(PIN_LED_DEADLINE, LOW);
    } 

    Serial.printf("[MPU] aX=%6d | gZ=%6d | exec=%lluus\n", 
                  accelerometer_x, gyro_z, execucao);
    
    // Espera pelo próximo período
    vTaskDelayUntil(&ativacao, periodTicks);
  }
}

// Tarefa Magnetômetro (HMC5883)
void taskmagnetrometro(void* param){
  const TickType_t periodTicks = pdMS_TO_TICKS(PERIOD_MAG_MS);
  TickType_t ativacao = xTaskGetTickCount();
  
  // Inicialização específica da tarefa HMC5883
  if (!mag.begin()){
    Serial.println("ERRO: HMC5883 não encontrado. Verifique a fiação!");
    vTaskDelete(NULL); // Deleta a tarefa se o sensor não for encontrado
    return;
  }
  
  while(true){
    uint64_t t0 = esp_timer_get_time();
    
    // Indica início da execução
    digitalWrite(PIN_LED_MAG, HIGH);
 
    // Leitura do HMC5883 (Protegida pelo Mutex I2C)
    sensors_event_t event;
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
      mag.getEvent(&event);
    xSemaphoreGive(i2cMutex);
    
    // Simulação de CPU Load para leitura
    busyWaitMicroseconds(computacao_mag);

    // Atualiza dados compartilhados
    xSemaphoreTake(dataMutex, portMAX_DELAY);
      sharedData.mag_x = (int32_t) event.magnetic.x;
      sharedData.mag_y = (int32_t) event.magnetic.y;
      sharedData.mag_z = (int32_t) event.magnetic.z;
      sharedData.lastMagTimeUs = esp_timer_get_time();
    xSemaphoreGive(dataMutex);

    // Indica fim da execução
    digitalWrite(PIN_LED_MAG, LOW);

    uint64_t t1 = esp_timer_get_time();
    uint64_t execucao = t1 - t0;

    // Checa se o deadline foi perdido
    if (execucao > PERIOD_MAG_MS * 1000) {
      Serial.printf("[MAG] Deadline PERDIDO! Exec=%lluus, Period=%dms\n", execucao, PERIOD_MAG_MS);
      digitalWrite(PIN_LED_DEADLINE, HIGH);
      delay(1);
      digitalWrite(PIN_LED_DEADLINE, LOW);
    }

    Serial.printf("[MAG] x=%ld y=%ld z=%ld | exec=%lluus\n",
                  sharedData.mag_x, sharedData.mag_y, sharedData.mag_z, execucao);

    // Espera pelo próximo período
    vTaskDelayUntil(&ativacao, periodTicks);
  }
}

// Tarefa Filtro (Exemplo: Média Móvel Simples)
void taskfiltro(void* param){
  const TickType_t periodTicks = pdMS_TO_TICKS(PERIOD_FILTRO_MS);
  TickType_t ativacao = xTaskGetTickCount();

  // Buffers de média móvel
  const int N = 5;
  int16_t accel_z_buf[N] = {0}; 
  int16_t gyro_z_buf[N] = {0};
  int idx = 0;
  int filled = 0;

  while(true){
    uint64_t t0 = esp_timer_get_time();

    // Indica início da execução
    digitalWrite(PIN_LED_FILTRO, HIGH);

    // Lê os dados brutos do acelerômetro e giroscópio
    int16_t l_accel_z, l_gyro_z;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
      l_accel_z = sharedData.accel_z;
      l_gyro_z = sharedData.gyro_z;
    xSemaphoreGive(dataMutex);

    // Insere no buffer
    accel_z_buf[idx] = l_accel_z; 
    gyro_z_buf[idx] = l_gyro_z;
    idx = (idx + 1) % N;
    filled = min(filled+1, N);

    // Calcula a média móvel (Simplesmente sobre o eixo Z para simplificar o exemplo)
    long sx=0, sy=0;
    for (int i=0;i<filled;i++){ 
      sx += accel_z_buf[i];
      sy += gyro_z_buf[i];
    }
    long avg_accel_z = sx / filled;
    long avg_gyro_z = sy / filled;

    // Simula CPU Load para o filtro
    busyWaitMicroseconds(computacao_filtro);

    // Atualiza o dado filtrado (opcional)
    xSemaphoreTake(dataMutex, portMAX_DELAY);
      sharedData.avg_accel_z = avg_accel_z;
      sharedData.avg_gyro_z = avg_gyro_z;
    xSemaphoreGive(dataMutex);

    // Indica fim da execução
    digitalWrite(PIN_LED_FILTRO, LOW);

    uint64_t t1 = esp_timer_get_time();
    uint64_t execucao = t1 - t0;

    // Checa se o deadline foi perdido
    if (execucao > PERIOD_FILTRO_MS * 1000) {
      Serial.printf("[FILT] Deadline PERDIDO! Exec=%lluus, Period=%dms\n", execucao, PERIOD_FILTRO_MS);
      digitalWrite(PIN_LED_DEADLINE, HIGH);
      delay(1);
      digitalWrite(PIN_LED_DEADLINE, LOW);
    }
    
    Serial.printf("[FILT] Avg_aZ=%ld | Avg_gZ=%ld | exec=%lluus\n",
                  avg_accel_z, avg_gyro_z, execucao);

    // Espera pelo próximo período
    vTaskDelayUntil(&ativacao, periodTicks);
  }
}

// --- SETUP E LOOP PRINCIPAL ---

void setup(){
  Serial.begin(115200);
  Serial.println("\n--- RTOS Sensor Fusion Project Initialized ---");
  
  // Configuração dos pinos I2C do ESP32 (GPIO 21/22 são os padrões, mas é bom especificar se necessário)
  // Wire.begin(SDA_PIN, SCL_PIN); 
  Wire.begin(); 

  // Configuração dos LEDs
  pinMode(PIN_LED_MAG, OUTPUT);
  pinMode(PIN_LED_MPU, OUTPUT);
  pinMode(PIN_LED_FILTRO, OUTPUT);
  pinMode(PIN_LED_DEADLINE, OUTPUT);

  // Inicialização do HMC5883 (o MPU6050 é inicializado dentro da sua Task)
  // Nota: mag.begin() usa Wire.begin() internamente.

  // Criação dos Mutexes
  i2cMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  // Criação das Tasks com prioridades escalonadas
  // Prioridade mais alta: Sensor mais rápido (MPU)
  xTaskCreate(taskacelerometro, "MPU", 4096, NULL, 3, NULL);      
  xTaskCreate(taskmagnetrometro, "MAG", 4096, NULL, 2, NULL); // Prioridade média
  xTaskCreate(taskfiltro, "FILTRO", 4096, NULL, 1, NULL);       // Prioridade mais baixa
}

void loop(){
  // loop() é usado pelo Idle Task do FreeRTOS.
  // Colocamos a tarefa para dormir infinitamente para não consumir ciclos.
  vTaskDelay(portMAX_DELAY);
}