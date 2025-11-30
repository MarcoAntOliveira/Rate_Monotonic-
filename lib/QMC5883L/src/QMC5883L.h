#ifndef QMC5883L_H
#define QMC5883L_H

#include <Arduino.h>
#include <Wire.h>

class QMC5883L {
public:
    QMC5883L(uint8_t address = 0x1E);

    bool begin(TwoWire &wirePort = Wire);
    bool read(int &x, int &y, int &z);

private:
    uint8_t _addr;
    TwoWire *_wire;
};

#endif
