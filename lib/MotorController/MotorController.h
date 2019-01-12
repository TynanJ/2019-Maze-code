#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Motor.h"
#include "Config.h"
#include "Common.h"

class MotorController
{
public:
    MotorController() {};
    void init(Motor* motor_1, Motor* motor_2, Motor* motor_3, Motor* motor_4);
    void Turn(int speed, int direction);
    void Move(int speed, int direction);
    void stop();
    
private:
    Motor motor_TL;
    Motor motor_TR;
    Motor motor_BL;
    Motor motor_BR;
};

#endif