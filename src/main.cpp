#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>

// Sensores
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_MPU6050 mpu;

// GPIOs
#define LED_A     2
#define LED_B     4

// Mutex para o barramento I2C
SemaphoreHandle_t i2cMutex;


// ------------------ TASK MAGNETÔMETRO ------------------
void taskMag(void *param){
    pinMode(LED_A, OUTPUT);

    while(true){
        digitalWrite(LED_A, HIGH);

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

        digitalWrite(LED_A, LOW);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}



// ------------------ TASK MPU6050 (GY-521) ------------------
void taskGY521(void *param){
    pinMode(LED_B, OUTPUT);

    sensors_event_t a, g, temp;

    while(true){
        digitalWrite(LED_B, HIGH);

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

        digitalWrite(LED_B, LOW);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}



// ----------------------- SETUP -----------------------
void setup(){
    Serial.begin(115200);
    Wire.begin(21,22);

    // Inicializa sensores
    if(!mag.begin()){
        Serial.println("Falha ao iniciar magnetometro!");
    }

    if(!mpu.begin()){
        Serial.println("Falha ao iniciar MPU6050!");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // Mutex do I2C
    i2cMutex = xSemaphoreCreateMutex();

    // Cria Tasks
    xTaskCreate(taskMag,   "MagTask",   4096, NULL, 1, NULL);
    xTaskCreate(taskGY521, "GY521Task", 4096, NULL, 2, NULL);
}



// ------------------------ LOOP ------------------------
void loop(){
    vTaskDelay(pdMS_TO_TICKS(1000));
}
