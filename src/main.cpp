#include "mbed.h"
#include "main.hpp"
#include <diffrobot.hpp>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


// Robot Name: amanita_augusta 

// ROS Setup
ros::NodeHandle node;
// Messages
geometry_msgs::Twist sendVelocityMsg;
std_msgs::Float32 leftWheelVelocity;
std_msgs::Float32 rightWheelVelocity;
// Callback
void receiveVelocityFunc(const geometry_msgs::Twist& msg);
void lRiseA();
void lChangeB();
void rRiseA();
void rChangeB();
void controlLoop();
void turnOff();
// Publisher
ros::Publisher sendVelocity("amanita_augusta_mbed/velocity", &sendVelocityMsg);
ros::Publisher sendLeftWheelVelocity("amanita_augusta_mbed/left_wheel_velocity", &leftWheelVelocity);
ros::Publisher sendRightWheelVelocity("amanita_augusta_mbed/right_wheel_velocity", &rightWheelVelocity);
// Subscriber
ros::Subscriber<geometry_msgs::Twist> receiveVelocity("amanita_augusta_pc/velocity", receiveVelocityFunc);
ros::Subscriber<std_msgs::Bool> reciveTurnOff("amanita_augusta_pc/enable", turnOff);
// Diff Robot
bra::DiffRobot AmanitaAugusta(WHEELS_RADIUS, WHEELS_RADIUS, LENGHT_WHEELS);

// mbed
Ticker mbedRate;
Timer mbedTimer; // 20Hz

int main() {
	AmanitaAugusta.setupMonsterDrivers( A0, L_PWM, L_CW, L_CCW,  R_ENABLE, R_PWM, R_CW, R_CCW);
	AmanitaAugusta.setupEncoders(L_PHASE_A, L_PHASE_B, lRiseA, lChangeB, L_RESOLUTION, 0, R_PHASE_A, R_PHASE_B, rRiseA, rChangeB, R_RESOLUTION, 1);
	AmanitaAugusta.setupController(L_KP, L_KI, L_KD, R_KP, R_KI, R_KD, RATE_MS);

	node.initNode();
	node.advertise(sendVelocity);
	node.advertise(sendLeftWheelVelocity);
	node.advertise(sendRightWheelVelocity);
	node.subscribe(receiveVelocity);
	
	mbedRate.attach(&controlLoop, RATE);
	mbedTimer.start();

	while(true) {
		if(mbedTimer.read() >= RATE/10){
			node.spinOnce();
			mbedTimer.reset();
		}
	}
}

void controlLoop(){
	AmanitaAugusta.run();

	sendVelocityMsg.linear.x = AmanitaAugusta.EncoderLeft->readPulse();
	sendVelocityMsg.angular.z = AmanitaAugusta.MotorLeft->getPWM()*100;//*(velocity+1);
	sendVelocity.publish(&sendVelocityMsg); // Linear and Angular Velocity

	leftWheelVelocity.data = AmanitaAugusta.getVelocity(bra::Encoder::LEFT);  	
	rightWheelVelocity.data = AmanitaAugusta.getVelocity(bra::Encoder::RIGHT); 	

	sendLeftWheelVelocity.publish(&leftWheelVelocity);	// Left Wheel Velocity
	sendRightWheelVelocity.publish(&rightWheelVelocity);// Right Wheel Velocity
	
}

void receiveVelocityFunc(const geometry_msgs::Twist& msg){
	AmanitaAugusta.setVelocity(msg.linear.x, msg.angular.z);
}

void turnOff(const std_msgs::Bool& msg){
	if(msg.data) AmanitaAugusta.enable();
	else AmanitaAugusta.disable();
}

void lRiseA(){
	AmanitaAugusta.EncoderLeft->risePhaseAEvent();
}

void lChangeB(){
	AmanitaAugusta.EncoderLeft->changePhaseBEvent();
}

void rRiseA(){
	AmanitaAugusta.EncoderRight->risePhaseAEvent();
}

void rChangeB(){
	AmanitaAugusta.EncoderRight->changePhaseBEvent();
}