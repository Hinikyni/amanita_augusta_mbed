#include <mbed.h>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

// Robot Struct
const float WHEELS_RADIUS = 0.031;
const float LENGHT_WHEELS = 0.175;

// Robot Monster Shield Pins
// Left Motor
const PinName L_ENABLE  = A0;
const PinName L_PWM     = D5;
const PinName L_CW      = D7;
const PinName L_CCW     = D8;
// Right Motor
const PinName R_ENABLE  = A1;
const PinName R_PWM     = D6;
const PinName R_CW      = D4;
const PinName R_CCW     = D9;

// Robot Encoder Pins
// Left Wheel
const PinName L_PHASE_A = PH_0;
const PinName L_PHASE_B = PH_1;
// Right Wheel
const PinName R_PHASE_A = PC_14;
const PinName R_PHASE_B = PC_15;
// Robot Encoder Settings
// Left Wheel
const int L_RESOLUTION = 300;
// Right Wheel
const int R_RESOLUTION = 300;

// Robot PID Consts
// Left Motor
const float L_KP = 1, L_KI = 0, L_KD = 0;
// Right Motor
const float R_KP = 1, R_KI = 0, R_KD = 0;

// Update
const float RATE_MS = 150;
const float RATE = RATE_MS/1000;

// ROS Setup
ros::NodeHandle node;
// Messages
geometry_msgs::Twist sendVelocityMsg;
std_msgs::Float32 leftWheelVelocity;
std_msgs::Float32 rightWheelVelocity;

// Callback
void receiveVelocityFunc(const geometry_msgs::Twist& msg);
void enableRobot(const std_msgs::Bool& msg);
void lRiseA();
void lChangeB();
void rRiseA();
void rChangeB();
void controlLoop();