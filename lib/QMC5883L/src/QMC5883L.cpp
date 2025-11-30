#include "QMC5883L.h"

QMC5883L::QMC5883L(uint8_t address) {
    _addr = address;
    _wire = &Wire;
}

bool QMC5883L::begin(TwoWire &wirePort) {
    _wire = &wirePort;

    _wire->beginTransmission(_addr);
    _wire->write(0x02);     // Register A: modo
    _wire->write(0x00);     // Continuous measurement mode
    if (_wire->endTransmission() != 0) {
        return false;       // Sensor não respondeu
    }
    return true;
}

bool QMC5883L::read(int &x, int &y, int &z) {
    // Aponta para o registrador de saída
    _wire->beginTransmission(_addr);
    _wire->write(0x03);
    if (_wire->endTransmission() != 0) {
        return false;
    }

    // Lê 6 bytes
    _wire->requestFrom(_addr, (uint8_t)6);
    if (_wire->available() < 6) return false;

    x = (_wire->read() << 8) | _wire->read();
    z = (_wire->read() << 8) | _wire->read();
    y = (_wire->read() << 8) | _wire->read();

    return true;
}
