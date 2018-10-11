#ifndef LED_H
#define LED_H


#include <Arduino.h>
#include "Config.h"


class LED {
public:
    LED(uint8_t pin);
    void setPower(uint8_t power);
private:
    uint8_t _pin;
};

#endif