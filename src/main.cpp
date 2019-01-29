#include "mbed.h"
#include "main.hpp"
#include <diffrobot.hpp>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

std::string robotName = "amanita_cecilia";

// Publisher [ _mbed/ ]
ros::Publisher sendVelocity("amanita_cecilia_mbed/velocity", &sendVelocityMsg);
ros::Publisher sendLeftWheelVelocity("amanita_cecilia_mbed/left_wheel_velocity", &leftWheelVelocity);
ros::Publisher sendRightWheelVelocity("amanita_cecilia_mbed/right_wheel_velocity", &rightWheelVelocity);
// Subscriber [ _pc/ ]
ros::Subscriber<std_msgs::Bool> reciveTurnOff("amanita_cecilia_pc/enable", enableRobot);
ros::Subscriber<geometry_msgs::Twist> receiveVelocity("amanita_cecilia_pc/cmd_vel", receiveVelocityFunc);
// Diff Robot
bra::DiffRobot Robot(WHEELS_RADIUS, WHEELS_RADIUS, LENGHT_WHEELS);

// Timers and Tickers
Ticker mbedRate;

int main() {
	Robot.setupMonsterDrivers( L_ENABLE, L_PWM, L_CW, L_CCW,  R_ENABLE, R_PWM, R_CW, R_CCW);
	Robot.setupEncoders(L_PHASE_A, L_PHASE_B, lRiseA, lChangeB, L_RESOLUTION, 0, R_PHASE_A, R_PHASE_B, rRiseA, rChangeB, R_RESOLUTION, 1);
	Robot.setupController(L_KP, L_KI, L_KD, R_KP, R_KI, R_KD, RATE_MS);

	node.initNode();
	node.advertise(sendVelocity);
	node.advertise(sendLeftWheelVelocity);
	node.advertise(sendRightWheelVelocity);
	node.subscribe(receiveVelocity);
	node.subscribe(reciveTurnOff);
	
	mbedRate.attach(&controlLoop, RATE);
	while(true) {
		node.spinOnce();
		// Disable robot motor if ROS connection has been lost
		if(!node.connected() && Robot.getStatus()) { Robot.setVelocity(0, 0); }
	}
}

void controlLoop(){
	Robot.run();

	volatile float* velocity = Robot.getVelocity();
	sendVelocityMsg.linear.x = velocity[bra::DiffRobot::LINEAR];
	sendVelocityMsg.angular.z = velocity[bra::DiffRobot::ANGULAR];
	
	sendVelocity.publish(&sendVelocityMsg); // Linear and Angular Velocity
	leftWheelVelocity.data = Robot.getVelocity(bra::Encoder::LEFT);  	
	rightWheelVelocity.data = Robot.getVelocity(bra::Encoder::RIGHT); 	
	
	sendLeftWheelVelocity.publish(&leftWheelVelocity);	// Left Wheel Velocity
	sendRightWheelVelocity.publish(&rightWheelVelocity);// Right Wheel Velocity
}

void receiveVelocityFunc(const geometry_msgs::Twist& msg){
	Robot.setVelocity(msg.linear.x, msg.angular.z);

}

void enableRobot(const std_msgs::Bool& msg){
	if(msg.data) Robot.enable();
	else Robot.disable();
}

void lRiseA(){
	Robot.EncoderLeft->risePhaseAEvent();
}

void lChangeB(){
	Robot.EncoderLeft->changePhaseBEvent();
}

void rRiseA(){
	Robot.EncoderRight->risePhaseAEvent();
}

void rChangeB(){
	Robot.EncoderRight->changePhaseBEvent();
}