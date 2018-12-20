#include <mbed.h>

#include "encoder.hpp"

bra::Encoder::Encoder(PinName p_phasePinA, PinName p_phasePinB, void (*CallbackRiseA)(), void (*CallbackChangeB)(), int p_resolution, bool p_side){
    
    m_phasePin[A] = new InterruptIn(p_phasePinA);
    m_phasePin[B] = new InterruptIn(p_phasePinB);
    m_side = p_side;
    m_resolution = p_resolution;

    m_phasePin[A]->rise(CallbackRiseA);     // Rise Event Phase A
    m_phasePin[B]->rise(CallbackChangeB);   // Change Event Phase B    
    m_phasePin[B]->fall(CallbackChangeB);   // Change Event Phase B
    
    m_phaseBValue = (bool)m_phasePin[B]->read();
    m_pulseCounter = 0;
    
    m_enableStatus = true;
}

bra::Encoder::~Encoder(){
    m_phasePin[A]->rise(NULL);    // Change Event Phase A
    m_phasePin[B]->rise(NULL);    // Change Event Phase B    
    m_phasePin[B]->fall(NULL);    // Change Event Phase B

    delete m_phasePin[A];
    delete m_phasePin[B];
}

void bra::Encoder::risePhaseAEvent(){
    if(m_enableStatus){
        if(m_phaseBValue){
            if(m_side) m_pulseCounter--;
            else m_pulseCounter++;
        } else {
            if(m_side) m_pulseCounter++;
            else m_pulseCounter--;
        }
    } else {
        m_pulseCounter = 0;
    }
}

void bra::Encoder::changePhaseBEvent(){
    m_phaseBValue = (bool)m_phasePin[B]->read();
}

void bra::Encoder::enable(){
    m_enableStatus = true;
}

void bra::Encoder::disable(){
    m_enableStatus = false;
}

int bra::Encoder::readPulse(){
    int pulse = m_pulseCounter;
    m_pulseCounter = 0;
    return pulse;
}

float bra::Encoder::readRotates(){
    if(m_resolution > 0){
        return this->readPulse()/m_resolution;
    } else {
        return 0;
    }
}

int bra::Encoder::getResolution(){
    return m_resolution;
}