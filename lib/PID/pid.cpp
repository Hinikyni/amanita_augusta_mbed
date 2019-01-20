#include <mbed.h>
#include "pid.hpp"

bra::PID::PID(double Kp, double Ki, double Kd, int interval){
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

double bra::PID::run(double input){
    double P,I,D, PIDR;

    //* Proportional Action - 비례 *//
    double pidError = _pidSetPoint - (input); 
    unsigned long pidDeltaTime;
    if(_lastTime == 0){
        _Timer.start(); // 
        pidDeltaTime = 0;
    } else {
        pidDeltaTime = _Timer.read_high_resolution_us() - _lastTime;
    }
    _lastTime = _Timer.read_high_resolution_us();
    
    //* Integral Action - 적분 *//
    _pidIntError += ( (pidError + _pidLastError)/2 * pidDeltaTime * 0.000001); 
    if(pidError == 0) _pidIntError = 0;
    if(_pidIntError > _maxWindUp){
        _pidIntError = _maxWindUp;
    } else if (_pidIntError < _minWindUp){
        _pidIntError = _minWindUp;
    }
    
    //* Diferencial Action - 미분*//
    double pidDifError; 
    if(pidDeltaTime) {
        pidDifError = (pidError - _pidLastError)  / (pidDeltaTime * 0.000001);
    } else {
        pidDifError = 0;
    }

    _pidLastError = pidError;

    P = (pidError) * _kP;
    I = _pidIntError * _kI;
    D = (pidDifError) * _kD;

    PIDR = P + I + D;
    
    return (PIDR);
}

void bra::PID::setSetpoint(double pidSetPoint){ //Set the Reference to PID
    _pidSetPoint = pidSetPoint;
}