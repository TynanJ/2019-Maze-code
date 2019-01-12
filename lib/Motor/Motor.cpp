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

void Motor::spin(int speed) 
{
    if(speed >= 0) {
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    } else {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    }


    speed = abs(speed);
    analogWrite(_en, speed);
}
