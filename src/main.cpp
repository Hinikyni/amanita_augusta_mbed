#include "mbed.h"
#include "main.hpp"
#include <diffrobot.hpp>
#include "ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <string>

std::string robotName = "amanita_augusta";

// Publisher
ros::Publisher sendVelocity((robotName+"_mbed/velocity").c_str(), &sendVelocityMsg);
ros::Publisher sendLeftWheelVelocity((robotName+"_mbed/left_wheel_velocity").c_str(), &leftWheelVelocity);
ros::Publisher sendRightWheelVelocity((robotName+"_mbed/right_wheel_velocity").c_str(), &rightWheelVelocity);
// Subscriber
ros::Subscriber<std_msgs::Bool> reciveTurnOff((robotName+"_pc/enable").c_str(), enableRobot);
ros::Subscriber<geometry_msgs::Twist> receiveVelocity((robotName+"_pc/cmd_vel").c_str(), receiveVelocityFunc);
// Diff Robot
bra::DiffRobot AmanitaAugusta(WHEELS_RADIUS, WHEELS_RADIUS, LENGHT_WHEELS);

// Timers and Tickers
Ticker mbedRate;
Timer mbedTimer; 

int main() {
	AmanitaAugusta.setupMonsterDrivers( A0, L_PWM, L_CW, L_CCW,  R_ENABLE, R_PWM, R_CW, R_CCW);
	AmanitaAugusta.setupEncoders(L_PHASE_A, L_PHASE_B, lRiseA, lChangeB, L_RESOLUTION, 0, R_PHASE_A, R_PHASE_B, rRiseA, rChangeB, R_RESOLUTION, 1);
	AmanitaAugusta.setupController(L_KP, L_KI, L_KD, R_KP, R_KI, R_KD, RATE_MS);

	node.initNode();
	node.advertise(sendVelocity);
	node.advertise(sendLeftWheelVelocity);
	node.advertise(sendRightWheelVelocity);
	node.subscribe(receiveVelocity);
	node.subscribe(reciveTurnOff);
	
	mbedRate.attach(&controlLoop, RATE);
	mbedTimer.start();

	while(true) {
		if(mbedTimer.read_ms() >= RATE_MS/10){
			node.spinOnce();
			mbedTimer.reset();
		}
	}
}

void controlLoop(){
	AmanitaAugusta.run();

	//! [FIXING] sendVelocityMsg.linear.x = AmanitaAugusta.EncoderLeft->readPulse();
	//! [FIXING] sendVelocityMsg.angular.z = AmanitaAugusta.MotorLeft->getPWM()*100;//*(velocity+1);
	sendVelocity.publish(&sendVelocityMsg); // Linear and Angular Velocity

	leftWheelVelocity.data = AmanitaAugusta.getVelocity(bra::Encoder::LEFT);  	
	rightWheelVelocity.data = AmanitaAugusta.getVelocity(bra::Encoder::RIGHT); 	

	sendLeftWheelVelocity.publish(&leftWheelVelocity);	// Left Wheel Velocity
	sendRightWheelVelocity.publish(&rightWheelVelocity);// Right Wheel Velocity
	
}

void receiveVelocityFunc(const geometry_msgs::Twist& msg){
	AmanitaAugusta.setVelocity(msg.linear.x, msg.angular.z);
}

void enableRobot(const std_msgs::Bool& msg){
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