#ifndef MANUFATURA_ADITIVA_PID_HPP
#define MANUFATURA_ADITIVA_PID_HPP
#include <mbed.h>

namespace bra{
    class PID{
        private:

            volatile float _kP, _kI, _kD;
            volatile double _pidSetPoint, 
                           _pidLastError,
                           _pidIntError;
            volatile unsigned long _lastTime;
            volatile float _maxWindUp, _minWindUp;
            Timer _Timer;

        public:
            int _interval;
            PID(float Kp, float Ki, float Kd, int interval);
            float run(double);
            void setSetpoint(float);
    };
}

#endif