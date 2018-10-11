#include "LightSensor.h"

LightSensor::LightSensor(uint8_t out) {
    _out = out;
}

uint32_t LightSensor::readLight() {
    return analogRead(_out);
}
