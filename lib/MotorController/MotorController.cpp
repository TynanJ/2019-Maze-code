#include "MotorController.h"


void MotorController::init(Motor* motor_1, Motor* motor_2, Motor* motor_3, Motor* motor_4) {
    motor_TL = *motor_2; 
    motor_TR = *motor_4;
    motor_BL = *motor_1; 
    motor_BR = *motor_3;
}

void MotorController::moveStraight(int rpm_left, int rpm_right, int direction) {
    bool new_direction = direction > 90;     // foward if direction > 90, else backwards

    motor_TL.set_direction(new_direction);
    motor_BL.set_direction(new_direction);
    motor_TR.set_direction(new_direction);
    motor_BR.set_direction(new_direction);


    motor_TL.set_speed(rpm_left * MOTOR2_OFFSET);
    motor_BL.set_speed(rpm_left * MOTOR1_OFFSET);
    motor_TR.set_speed(rpm_right * MOTOR4_OFFSET);
    motor_BR.set_speed(rpm_right * MOTOR3_OFFSET);

    // Serial.print(calculatePower(rpm_left, mA_a, mA_b));
    // Serial.print(',');
    // Serial.print(calculatePower(rpm_left, mB_a, mB_b));
    // Serial.print(',');
    // Serial.print(calculatePower(rpm_left, mC_a, mC_b));
    // Serial.print(',');
    // Serial.print(calculatePower(rpm_left, mD_a, mD_b));
    // Serial.println();
    
    
    // motor_TL.set_speed(calculatePower(rpm_left, mB_a, mB_b));
    // motor_BL.set_speed(calculatePower(rpm_left, mA_a, mA_b));
    // motor_TR.set_speed(calculatePower(rpm_right, mD_a, mD_b));
    // motor_BR.set_speed(calculatePower(rpm_right, mC_a, mC_b));

}

void MotorController::moveSideways(int rpm_left, int rpm_right, int direction) {
    bool new_direction = direction < 180;            // Left if direction < 180, else right

    motor_TL.set_direction(!new_direction);
    motor_BL.set_direction(new_direction);
    motor_TR.set_direction(new_direction);
    motor_BR.set_direction(!new_direction);
    
    motor_TL.set_speed(rpm_left * MOTOR2_OFFSET * MOTOR2_SIDEWAYS_OFFSET);
    motor_BL.set_speed(rpm_left * MOTOR1_OFFSET * MOTOR1_SIDEWAYS_OFFSET);
    motor_TR.set_speed(rpm_right* MOTOR4_OFFSET * MOTOR4_SIDEWAYS_OFFSET);
    motor_BR.set_speed(rpm_right* MOTOR3_OFFSET * MOTOR3_SIDEWAYS_OFFSET);
}

void MotorController::turn(int rpm_left, int rpm_right, bool direction) {
    motor_TL.set_direction(direction);
    motor_BL.set_direction(direction);
    motor_TR.set_direction(!direction);
    motor_BR.set_direction(!direction);
    
    motor_TL.set_speed(rpm_left * MOTOR2_OFFSET);
    motor_BL.set_speed(rpm_left * MOTOR1_OFFSET);
    motor_TR.set_speed(rpm_right * MOTOR4_OFFSET);
    motor_BR.set_speed(rpm_right * MOTOR3_OFFSET);
    
}

void MotorController::moveAll(int rpm_left, int rpm_right, int direction) {
    if (direction % 180 == 0) {
        moveStraight(rpm_left, rpm_right, direction);
    } else {
        moveSideways(rpm_left, rpm_right, direction);
    }
}


void MotorController::stop() {
    motor_TL.set_speed(0);
    motor_BL.set_speed(0);
    motor_TR.set_speed(0);
    motor_BR.set_speed(0);
}

double MotorController::calculatePower(uint8_t rpm, double a, double b) {
    return exp((rpm - a) / b);
}