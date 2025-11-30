#ifndef MAG_H
#define MAG_H

#include <Arduino.h>
#include <Wire.h>
#include "QMC5883L/QMC5883L.h"
    QMC5883L mag(0x0D);
    SemaphoreHandle_t i2cMutex;
    QueueHandle_t magQueue;

    void taskMag(void *pvParameters);
    void taskProcessMag(void *pvParameters) ;

#endif
