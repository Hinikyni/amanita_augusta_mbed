#ifndef MANUFATURA_ADITIVA_PID_HPP
#define MANUFATURA_ADITIVA_PID_HPP
#include <mbed.h>

namespace bra{
    class PID{
        private:

            volatile double _kP, _kI, _kD;
            volatile double _pidSetPoint, 
                           _pidLastError,
                           _pidIntError;
            volatile unsigned long _lastTime;
            volatile double _maxWindUp, _minWindUp;
            Timer _Timer;

        public:
            int _interval;
            PID(double Kp, double Ki, double Kd, int interval);
            double run(double);
            void setSetpoint(double);
    };
}

#endif