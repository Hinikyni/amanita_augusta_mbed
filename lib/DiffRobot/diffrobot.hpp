#ifndef MANUFATURA_ADITIVA_DIFFROBOT_HPP
#define MANUFATURA_ADITIVA_DIFFROBOT_HPP

#include <mbed.h>
#include <monsterdriver.hpp>
#include <encoder.hpp>
#include <PID.h>

namespace bra{

    class DiffRobot {

        private:
            MonsterDriver* MotorLeft;
            MonsterDriver* MotorRight;
            
            Encoder* EncoderLeft;
            Encoder* EncoderRight;
            
            PID* VelocityControllerLeft;
            PID* VelocityControllerRight;

            float m_wheelLeftRadius, m_wheelRightRadius; // Wheel's Radius
            float m_lengthWheels; // Distance beetween wheels
            float m_velocity[2]; // Linear and Angular Velocity
            float m_wheelsVelocityTarget[2]; // Desired Wheels Velocity
            float m_wheelsVelocity[2]; // Wheels Velocity
            
            const float m_PI = 3.14159265359;
            enum {LINEAR, ANGULAR};
        public:
            DiffRobot(float p_wheelLeftRadius, float p_wheelRightRadius, float p_lengthWheels);
            ~DiffRobot();

            void setupMonsterDrivers(   // Left Motor
                                        PinName p_motorLeftEnablePin,
                                        PinName p_motorLeftPwmPin, 
                                        PinName p_motorLeftDirPin_A,
                                        PinName p_motorLeftDirPin_B,
                                        // Right Motor
                                        PinName p_motorRightEnablePin,
                                        PinName p_motorRightPwmPin, 
                                        PinName p_motorRightDirPin_A,
                                        PinName p_motorRightDirPin_B                                        
                                    );
            void setupEncoders( // Left Wheel
                                PinName p_phasePinALeft, PinName p_phasePinBLeft, void (*callbackRiseALeft)(), void (*callbackChangeBLeft)(), int p_resolutionLeft, bool p_sideLeft,
                                // Right Wheel
                                PinName p_phasePinARight, PinName p_phasePinBRight, void (*callbackRiseARight)(), void (*callbackChangeBRight)(), int p_resolutionRight, bool p_sideRight
                            );
            void setupController(   // Left Motor
                                    float p_KpLeft, float p_KiLeft, float p_KdLeft,
                                    // Right Motor
                                    float p_KpRight, float p_KiRight, float p_KdRight,
                                    float p_interval
                                );

            void setVelocity(float p_linear, float p_angular);   // Set Velocity in m/s and rad/s
            float* getVelocity();   // Return velocity in m/s and rad/s
            void run();
    };
}
#endif