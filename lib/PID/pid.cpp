#include <mbed.h>
#include "pid.hpp"

bra::PID::PID(float Kp, float Ki, float Kd, int interval){
    _kP = Kp;
    _kI = Ki;
    _kD = Kd;
    _interval = interval;

    _pidSetPoint = 0,
    _pidLastError = 0,
    _pidIntError = 0;
    _lastTime = 0;
    _maxWindUp = 1.0;
    _minWindUp = -1.0;
}

float bra::PID::run(double input){
    float P,I,D, PIDR;

    //* Proportional Action - 비례 *//
    double pidError = this->_pidSetPoint - (input); 
    unsigned long pidDeltaTime;
    if(_lastTime == 0){
        _Timer.start(); // 
        pidDeltaTime = 0;
    } else {
        pidDeltaTime = _Timer.read_high_resolution_us() - this->_lastTime;
    }
    this->_lastTime = _Timer.read_high_resolution_us();
    
    //* Integral Action - 적분 *//
    this->_pidIntError += (pidError * pidDeltaTime * 0.000001); 

    if(this->_pidIntError > this->_maxWindUp){
        this->_pidIntError = this->_maxWindUp;
    } else if (this->_pidIntError < this->_minWindUp){
        this->_pidIntError = this->_minWindUp;
    }
    
    //* Diferencial Action - 미분*//
    float pidDifError; 
    if(pidDeltaTime) {
        pidDifError = (pidError - this->_pidLastError)  / (pidDeltaTime * 0.000001);
    } else {
        pidDifError = 0;
    }

    this->_pidLastError = pidError;

    P = (pidError) * this->_kP;
    I = this->_pidIntError * this->_kI;
    D = (pidDifError) * this->_kD;

    PIDR = P + I + D;
    
    return (PIDR);
}

void bra::PID::setSetpoint(float pidSetPoint){ //Set the Reference to PID
    this->_pidSetPoint = pidSetPoint;
}