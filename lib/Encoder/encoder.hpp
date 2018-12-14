#ifndef MANUFATURA_ADITIVA_ENCODER_HPP
#define MANUFATURA_ADITIVA_ENCODER_HPP

#include <mbed.h>

namespace bra{

    class Encoder{
        private:
            InterruptIn* m_phasePin[2];         // Phase A and Phase B Pins
            volatile unsigned long m_pulseCounter;   // Pulses Counted
            volatile int m_resolution;
            volatile bool m_phaseBValue;           // Last read value of A and B
            volatile bool m_side;                    // Left or Right side
            volatile bool m_enableStatus;
            enum {A, B};
        public:
            enum {LEFT, RIGHT};
            Encoder(PinName p_phasePinA, PinName p_phasePinB, void (*callbackRiseA)(), void (*callbackChangeB)(), int p_resolution = 1, bool p_side = LEFT);
            ~Encoder(void);
            void enable(); // Default
            void disable();
            int readPulse();
            float readRotates();
            int getResolution();
            void risePhaseAEvent();
            void changePhaseBEvent();
    };


}
#endif