#include "mag.h"

void setup() {
    Serial.begin(115200);

    Wire.begin(21, 22, 400000); // SDA, SCL, Clock

    i2cMutex = xSemaphoreCreateMutex();
    magQueue = xQueueCreate(1, sizeof(struct MagData));

    xTaskCreatePinnedToCore(taskMag, "MagTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskProcessMag, "ProcMag", 4096, NULL, 1, NULL, 1);
}

void loop() {}
