#include "LED.h"


LED::LED(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
}

void LED::setPower(uint8_t power) {
    analogWrite(_pin, power);
}
