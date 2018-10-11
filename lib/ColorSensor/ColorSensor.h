#ifndef COLOR_H
#define COLOR_H

#include <Arduino.h>
#include "Common.h"
#include "Config.h"

class ColorSensor
{
public:
    ColorSensor() {};
    void init(uint8_t S0, uint8_t S1, uint8_t S2, uint8_t S3, uint8_t sensorOut);
    void calibrate();
    IntVector3D readColor();
    double get_BG_RED();
    double get_BG_GREEN();
    double get_BG_BLUE();


private:
    uint8_t _S0;
    uint8_t _S1;
    uint8_t _S2;
    uint8_t _S3;
    uint8_t _sensorOut;

    double _BG_RED;
    double _BG_GREEN;
    double _BG_BLUE;

    int frequencyRed;
    int frequencyBlue;
    int frequencyGreen;
};

#endif