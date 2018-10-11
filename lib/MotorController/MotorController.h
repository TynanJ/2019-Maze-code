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
    void moveStraight(int rpm_left, int rpm_right, int direction);
    void moveSideways(int rpm_left, int rpm_right, int direction);
    void turn(int rpm_left, int rpm_right, bool direction);
    void moveAll(int rpm_left, int rpm_right, int direction);
    void stop();
    double calculatePower(uint8_t rpm, double a, double b);

private:
    Motor motor_TL;
    Motor motor_TR;
    Motor motor_BL;
    Motor motor_BR;
};

#endif