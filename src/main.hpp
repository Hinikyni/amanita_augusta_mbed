#include <mbed.h>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

// Robot Struct
const float WHEELS_RADIUS = 0.0325; // [m]
const float LENGHT_WHEELS = 0.156;  // [m]

// Robot Monster Shield Pins
// Left Motor
const PinName L_ENABLE  = A1;
const PinName L_PWM     = D6;
const PinName L_CW      = D4;
const PinName L_CCW     = D9;
// Right Motor
const PinName R_ENABLE  = A0;
const PinName R_PWM     = D5;
const PinName R_CW      = D7;
const PinName R_CCW     = D8;

// Robot Encoder Pins
// Left Wheel
const PinName L_PHASE_A = PC_6;
const PinName L_PHASE_B = PC_8;
// Right Wheel
const PinName R_PHASE_A = PB_1;
const PinName R_PHASE_B = PB_2;
// Robot Encoder Settings
// Left Wheel
const float L_RESOLUTION = 374;
// Right Wheel
const float R_RESOLUTION = 374;

// Robot PID Consts
// Left Motor
const float L_KP = 0.9, L_KI = 10.56, L_KD = 0.0;
// Right Motor
const float R_KP = 0.9, R_KI = 11.16, R_KD = 0.0;

// Update
const float RATE_MS = 50;  // [ms]
const float RATE = RATE_MS/1000; // [s]
const float MS2SEC = 0.001;
// ROS Setup
ros::NodeHandle node;
// Messages
geometry_msgs::Twist sendVelocityMsg;
std_msgs::Float32 leftWheelVelocity;    // [m/s]
std_msgs::Float32 rightWheelVelocity;   // [m/s]
// Callback
void receiveVelocityFunc(const geometry_msgs::Twist& msg);
void enableRobot(const std_msgs::Bool& msg);
void lRiseA();
void lChangeB();
void rRiseA();
void rChangeB();
void controlLoop();