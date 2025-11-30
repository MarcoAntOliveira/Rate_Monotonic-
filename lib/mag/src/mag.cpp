#include "mag.h"
void taskMag(void *pvParameters) {
   

    // Inicializa o sensor com mutex
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    bool ok = mag.begin(Wire);
    xSemaphoreGive(i2cMutex);

    if (!ok) {
        Serial.println("[MAG] Falha ao iniciar sensor!");
        vTaskDelete(NULL);
    }

    Serial.println("[MAG] Sensor OK!");

    struct MagData {
        int x, y, z;
    } data;

    while (true) {

        xSemaphoreTake(i2cMutex, portMAX_DELAY);
        bool status = mag.read(data.x, data.y, data.z);
        xSemaphoreGive(i2cMutex);

        if (status) {
            Serial.printf("[MAG] X:%d  Y:%d  Z:%d\n", data.x, data.y, data.z);
            xQueueOverwrite(magQueue, &data);
        } else {
            Serial.println("[MAG] Erro na leitura!");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void taskProcessMag(void *pvParameters) {

    struct MagData { int x, y, z; } data;

    while (true) {

        if (xQueueReceive(magQueue, &data, portMAX_DELAY)) {

            float heading = atan2(data.y, data.x) * 180.0 / PI;
            if (heading < 0) heading += 360;

            Serial.printf("[PROC] heading: %.2f graus\n", heading);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
