#include <mbed.h>

#include "diffrobot.hpp"
#include <monsterdriver.hpp>
#include <encoder.hpp>
#include <PID.h>
#include <string>

bra::DiffRobot::DiffRobot(float p_wheelLeftRadius, float p_wheelRightRadius, float p_lengthWheels){
    // Initial Variables
    m_wheelLeftRadius = p_wheelLeftRadius;
    m_wheelRightRadius = p_wheelRightRadius;
    m_lengthWheels = p_lengthWheels;
    m_robotStatus = true;
    // Default Velocity
    m_velocity[LINEAR] = 0;
    m_velocity[ANGULAR] = 0;
    m_wheelsVelocity[Encoder::LEFT] = 0;
    m_wheelsVelocity[Encoder::RIGHT] = 0;
    // Default SetPoint
    m_wheelsVelocityTarget[Encoder::LEFT] = 0;
    m_wheelsVelocityTarget[Encoder::RIGHT] = 0;
};

bra::DiffRobot::~DiffRobot(){
    MotorLeft->setPWM(0.0f);
    MotorRight->setPWM(0.0f);
};

void bra::DiffRobot::setupMonsterDrivers(    // Left Motor
                                        PinName p_motorLeftEnablePin,
                                        PinName p_motorLeftPwmPin, 
                                        PinName p_motorLeftDirPin_A,
                                        PinName p_motorLeftDirPin_B,
                                        // Right Motor
                                        PinName p_motorRightEnablePin,
                                        PinName p_motorRightPwmPin, 
                                        PinName p_motorRightDirPin_A,
                                        PinName p_motorRightDirPin_B
                                                                        )
{

    MotorLeft = new MonsterDriver(p_motorLeftEnablePin, p_motorLeftPwmPin, p_motorLeftDirPin_A, p_motorLeftDirPin_B);
    MotorRight = new MonsterDriver(p_motorRightEnablePin, p_motorRightPwmPin, p_motorRightDirPin_A, p_motorRightDirPin_B);
}

void bra::DiffRobot::setupEncoders(PinName p_phasePinALeft, PinName p_phasePinBLeft, void (*callbackRiseALeft)(), void (*callbackChangeBLeft)(), int p_resolutionLeft, bool p_sideLeft,
                                    PinName p_phasePinARight, PinName p_phasePinBRight, void (*callbackRiseARight)(), void (*callbackChangeBRight)(), int p_resolutionRight, bool p_sideRight
                                ) 
{
    EncoderLeft = new Encoder(p_phasePinALeft, p_phasePinBLeft, callbackRiseALeft, callbackChangeBLeft, p_resolutionLeft, Encoder::LEFT);
    EncoderRight = new Encoder(p_phasePinARight, p_phasePinBRight, callbackRiseARight, callbackChangeBRight, p_resolutionRight, Encoder::RIGHT);
}

void bra::DiffRobot::setupController(float p_KpLeft, float p_KiLeft, float p_KdLeft, float p_KpRight, float p_KiRight, float p_KdRight, float p_interval) {
    
    VelocityControllerLeft = new PID(p_KpLeft, p_KiLeft, p_KdLeft, p_interval*0.001);
    VelocityControllerRight = new PID(p_KpRight, p_KiRight, p_KdRight, p_interval*0.001);

    VelocityControllerLeft->setOutputLimits(-1.0, 1.0);
    VelocityControllerRight->setOutputLimits(-1.0, 1.0);
    VelocityControllerLeft->setInputLimits(-1.0, 1.0);
    VelocityControllerRight->setInputLimits(-1.0, 1.0);
}

void bra::DiffRobot::setVelocity(float p_linear, float p_angular){
    m_wheelsVelocityTarget[Encoder::LEFT] =  p_linear;//(p_linear + p_angular * m_lengthWheels * 0.5)/m_wheelLeftRadius;
    m_wheelsVelocityTarget[Encoder::RIGHT] = p_angular;//(p_linear - p_angular * m_lengthWheels * 0.5)/m_wheelRightRadius;
    
    VelocityControllerLeft->setSetPoint(m_wheelsVelocity[Encoder::LEFT]);
    VelocityControllerRight->setSetPoint(m_wheelsVelocity[Encoder::RIGHT]);
}

volatile float* bra::DiffRobot::getVelocity(){
    return m_velocity;
}

float bra::DiffRobot::getVelocity(int wheel){
    return m_wheelsVelocity[wheel];
}

void bra::DiffRobot::enable(){
    MotorLeft->enable();
    MotorRight->enable();
    m_robotStatus = true;
}

void bra::DiffRobot::disable(){
    MotorLeft->disable();
    MotorRight->disable();
    m_robotStatus = false;
}

bool bra::DiffRobot::getStatus(){
    return m_robotStatus;
}
void bra::DiffRobot::run(){
    // Update variables
    //? Wheel Speed = (2*pi*R)*(Pulses Counted / EncoderResolution) / (IntervalSinceLastCount)
    m_wheelsVelocity[Encoder::LEFT] =  ((2 * m_PI * m_wheelLeftRadius) *  EncoderLeft->readPulse()) / (VelocityControllerLeft->getInterval() * EncoderLeft->getResolution());
    m_wheelsVelocity[Encoder::RIGHT] = ((2 * m_PI * m_wheelRightRadius) * EncoderRight->readPulse()) / (VelocityControllerRight->getInterval() * EncoderRight->getResolution());
    m_velocity[LINEAR]  = ((m_wheelsVelocity[Encoder::LEFT]) + (m_wheelsVelocity[Encoder::RIGHT])) / 2;
    m_velocity[ANGULAR] = ((m_wheelsVelocity[Encoder::LEFT]) - (m_wheelsVelocity[Encoder::RIGHT])) / m_lengthWheels;
    /* //![FIXING] Modeling the system for PID controller.
    // Feedback PID controller
    VelocityControllerLeft->setProcessValue(m_wheelsVelocity[Encoder::LEFT]);
    VelocityControllerRight->setProcessValue(m_wheelsVelocity[Encoder::RIGHT]);

    // Compute the PID controller
    float controllerOutput[2];
    controllerOutput[Encoder::LEFT] = VelocityControllerLeft->compute();
    controllerOutput[Encoder::RIGHT] = VelocityControllerRight->compute();
    */
    // Set new PWM
    MotorLeft->setPWM(  m_wheelsVelocityTarget[0] );
    MotorRight->setPWM( m_wheelsVelocityTarget[1] );
}