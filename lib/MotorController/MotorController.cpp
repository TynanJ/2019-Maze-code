#include "MotorController.h"

void MotorController::init(Motor* motor_1, Motor* motor_2, Motor* motor_3, Motor* motor_4)
{
    motor_TL = *motor_2; 
    motor_TR = *motor_4;
    motor_BL = *motor_1; 
    motor_BR = *motor_3;
}

void MotorController::Turn(int speed, int direction)
{
    // Positive speed is a clockwise Turn
    motor_TL.spin(speed);
    motor_TR.spin(-speed);
    motor_BL.spin(speed);
    motor_BR.spin(-speed);


}

void MotorController::Move(int speed, int direction)
{

}

void MotorController::stop(){
    motor_TL.spin(0);
    motor_TR.spin(0);
    motor_BL.spin(0);
    motor_BR.spin(0);
}