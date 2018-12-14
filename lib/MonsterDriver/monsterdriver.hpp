#ifndef MANUFATURA_ADITIVA_MONSTERDRIVER_HPP
#define MANUFATURA_ADITIVA_MONSTERDRIVER_HPP

#include <mbed.h>

namespace bra{

    class MonsterDriver {
        private:
            DigitalOut* m_motorEnablePin;
            DigitalOut* m_motorDirPin_A;
            DigitalOut* m_motorDirPin_B;
            PwmOut* m_motorPwmPin;

            volatile float m_pwmValue {0.0f};
            const int m_pwmPeriod {50}; // us [ Frequency: 20kHz ]
            
        public:
            enum {LOW, HIGH};
            enum Direction {FORWARD, BACKWARD};
            MonsterDriver(  
                            PinName p_motorEnablePin, 
                            PinName p_motorPwmPin, 
                            PinName p_motorDirPin_A, 
                            PinName p_motorDirPin_B
                        );
            ~MonsterDriver(void);
            // Set the ouput duty-cycle, specified as a percentage (float)
            void setPWM(float p_pwmValue);
            float getPWM();
            // Set the direction
            void setDirection(Direction p_direction);
            // Enable or Disable
            void enable();
            void disable();
    };
}
#endif