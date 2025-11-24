#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

// ===================== OBJETOS ======================
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag(12345);

// ===================== FILAS ========================
QueueHandle_t filaBruta;
QueueHandle_t filaFiltrada;
QueueHandle_t filaAlerta;

// ===================== STRUCT DOS DADOS =============
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float heading;
} SensoresBrutos;

typedef struct {
    float ax, ay, az;
    float heading;
} SensoresFiltrados;

typedef struct {
    bool risco;
    String motivo;
} Alerta;

// ====================================================
//                  TASK 1 - SENSOR FUSION
// ====================================================
void taskFusion(void *p){
    sensors_event_t a, g, temp, magEvent;

    while(true){
        mpu.getEvent(&a, &g, &temp);
        mag.getEvent(&magEvent);

        SensoresBrutos d;

        d.ax = a.acceleration.x;
        d.ay = a.acceleration.y;
        d.az = a.acceleration.z;

        d.gx = g.gyro.x;
        d.gy = g.gyro.y;
        d.gz = g.gyro.z;

        d.heading = atan2(magEvent.magnetic.y, magEvent.magnetic.x) * 180 / M_PI;

        xQueueSend(filaBruta, &d, 0);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// ====================================================
//           TASK 2 - SEGURANÇA (heading instável)
// ====================================================
void taskSeguranca(void *p){
    SensoresBrutos d;

    const float CAMPO_MAX_VARIACAO = 20;
    float ultimoHeading = 0;

    while(true){
        if(xQueueReceive(filaBruta, &d, 10)){
            Alerta alerta;
            alerta.risco = false;

            if(abs(d.heading - ultimoHeading) > CAMPO_MAX_VARIACAO){
                alerta.risco = true;
                alerta.motivo = "Campo magnetico instavel!";
            }

            if(alerta.risco)
                xQueueSend(filaAlerta, &alerta, 0);

            ultimoHeading = d.heading;
        }
    }
}

// ====================================================
//           TASK 3 - FILTRO (média móvel)
// ====================================================
void taskFiltro(void *p){
    SensoresBrutos d;
    SensoresFiltrados f;

    const int N = 5;
    static float histHead[5] = {0};
    int idx = 0;

    while(true){
        if(xQueueReceive(filaBruta, &d, 10)){
            histHead[idx] = d.heading;
            idx = (idx + 1) % N;

            float somaHead = 0;
            for(int i=0;i<N;i++){
                somaHead += histHead[i];
            }

            f.ax = d.ax;
            f.ay = d.ay;
            f.az = d.az;
            f.heading = somaHead / N;

            xQueueSend(filaFiltrada, &f, 0);
        }
    }
}

// ====================================================
//        TASK 4 - DETECÇÃO DE MUDANÇA DE ORIENTAÇÃO
// ====================================================
void taskDeteccao(void *p){
    SensoresFiltrados d;

    float ultimoHead = 0;
    const float LIMIAR_ROT = 25;

    while(true){
        if(xQueueReceive(filaFiltrada, &d, 10)){
            float dHead = abs(d.heading - ultimoHead);

            if(dHead > LIMIAR_ROT){
                Alerta alerta;
                alerta.risco = true;
                alerta.motivo = "Mudanca brusca de orientacao!";
                xQueueSend(filaAlerta, &alerta, 0);
            }

            ultimoHead = d.heading;
        }
    }
}

// ====================================================
//             TASK 5 - PUBLICAÇÃO SERIAL
// ====================================================
void taskSerial(void *p){
    SensoresFiltrados d;
    Alerta alerta;

    while(true){
        if(xQueueReceive(filaFiltrada, &d, 5)){
            Serial.printf("[FILTRADO] Heading=%.2f°  Axyz=(%.2f, %.2f, %.2f)\n",
                          d.heading, d.ax, d.ay, d.az);
        }

        if(xQueueReceive(filaAlerta, &alerta, 5)){
            if(alerta.risco && alerta.motivo.length() > 0) {
                Serial.println("ALERTA: " + alerta.motivo);
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


// ====================================================
//                       SETUP
// ====================================================
void setup(){
    Serial.begin(115200);
    Wire.begin();

    mpu.begin();
    mag.begin();

    filaBruta = xQueueCreate(10, sizeof(SensoresBrutos));
    filaFiltrada = xQueueCreate(10, sizeof(SensoresFiltrados));
    filaAlerta = xQueueCreate(10, sizeof(Alerta));

    xTaskCreate(taskFusion,    "fusion",    4096, NULL, 2, NULL);
    xTaskCreate(taskSeguranca, "segur",     4096, NULL, 3, NULL);
    xTaskCreate(taskFiltro,    "filtro",    4096, NULL, 1, NULL);
    xTaskCreate(taskDeteccao,  "detec",     4096, NULL, 2, NULL);
    xTaskCreate(taskSerial,    "serial",    4096, NULL, 1, NULL);
}

void loop(){}
