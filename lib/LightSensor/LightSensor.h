#ifndef LIGHT_H
#define LIGHT_H

#include <Arduino.h>
#include "Config.h"

class LightSensor
{
public:
    LightSensor(uint8_t out);
    uint32_t readLight();
    void calibrate();

private:
    uint8_t _out;
   
};

#endif