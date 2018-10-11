#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor
{
private:
    int _en;
    int _in1;
    int _in2;

public:
    Motor() {};
    void init(int en, int in1, int in2);
    void set_direction(bool dir);
    void set_speed(int speed);
};

#endif      // MOTOR_H