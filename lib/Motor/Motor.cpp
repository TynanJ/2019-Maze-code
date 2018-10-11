#include "Motor.h"


void Motor::init(int en, int in1, int in2)
{
    _en = en;
    _in1 = in1;
    _in2 = in2;

    pinMode(_en, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
}

void Motor::set_direction(bool dir=true)
{
    if (!dir) {      // Foward movement
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    } else {        // Reverse movement
        digitalWrite(_in2, HIGH);
        digitalWrite(_in1, LOW);
    }
}


void Motor::set_speed(int speed)
{
    speed = abs(speed);
    analogWrite(_en, speed);
}