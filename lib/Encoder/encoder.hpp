#ifndef MANUFATURA_ADITIVA_ENCODER_HPP
#define MANUFATURA_ADITIVA_ENCODER_HPP

#include <mbed.h>

namespace bra{

    class Encoder{
        private:
            InterruptIn* m_phasePin[2];         // Phase A and Phase B Pins
            unsigned long m_pulseCounter;   // Pulses Counted
            int m_resolution;
            bool m_phaseBValue;           // Last read value of A and B
            bool m_side;                    // Left or Right side
            bool m_enableStatus;
            enum {A, B};
            void risePhaseAEvent();
            void changePhaseBEvent();
        public:
            enum {LEFT, RIGHT};
            Encoder(PinName p_phasePinA, PinName p_phasePinB, void (*callbackRiseA)(), void (*callbackChangeB)(), int p_resolution = 1, bool p_side = LEFT);
            ~Encoder(void);
            void enable(); // Default
            void disable();
            int readPulse();
            float readRotates();
    };


}
#endif