#include "ColorSensor.h"

void ColorSensor::init(uint8_t S0, uint8_t S1, uint8_t S2, uint8_t S3, uint8_t sensorOut) {
   _S0 = S0;
   _S1 = S1;
   _S2 = S2;
   _S3 = S3;
   _sensorOut = sensorOut;

    pinMode(_S0, OUTPUT);
    pinMode(_S1, OUTPUT);
    pinMode(_S2, OUTPUT);
    pinMode(_S3, OUTPUT);
    pinMode(_sensorOut, INPUT);

    digitalWrite(_S0, LOW);
    digitalWrite(_S1, HIGH);

    // ColorSensor::calibrate();
}


IntVector3D ColorSensor::readColor() {
    noInterrupts();
    // Setting red filtered photodiodes to be read
    digitalWrite(_S2,LOW);
    digitalWrite(_S3,LOW);

    // Reading the output frequency
    frequencyRed = pulseIn(_sensorOut, HIGH, 20000);

    // Setting Green filtered photodiodes to be read
    digitalWrite(_S2,HIGH);
    digitalWrite(_S3,HIGH);

    // Reading the output frequency
    frequencyGreen = pulseIn(_sensorOut, HIGH, 20000);
    
    // Setting Blue filtered photodiodes to be read
    digitalWrite(_S2,LOW);
    digitalWrite(_S3,HIGH);

    // Reading the output frequency
    frequencyBlue = pulseIn(_sensorOut, HIGH, 20000);
    
    interrupts();
    
    IntVector3D returnVector = {frequencyRed, frequencyGreen, frequencyBlue};
    return returnVector;
} 

void ColorSensor::calibrate() {
    for(int i=0; i < CALIBRATION_COUNT; i++) {
        IntVector3D temp = ColorSensor::readColor();

        _BG_RED += temp.x;
        _BG_GREEN += temp.y;
        _BG_BLUE += temp.z;

        delay(10);
    }

    _BG_RED /= CALIBRATION_COUNT; 
    _BG_GREEN /= CALIBRATION_COUNT;
    _BG_BLUE /= CALIBRATION_COUNT;
}

double ColorSensor::get_BG_RED() {
    return _BG_RED;
}

double ColorSensor::get_BG_GREEN() {
    return _BG_GREEN;
}

double ColorSensor::get_BG_BLUE() {
    return _BG_BLUE;
}
