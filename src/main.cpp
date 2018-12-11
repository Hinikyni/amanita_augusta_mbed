#include "mbed.h"
#include "main.hpp"
#include <diffrobot.hpp>
#include "ros.h"
#include <geometry_msgs/Twist.h>

// Robot Name: amanita_augusta 

// ROS Setup
ros::NodeHandle node;
// Messages
geometry_msgs::Twist sendVelocityMsg;
// Callback
void receiveVelocityFunc(const geometry_msgs::Twist& msg);
void lRiseA();
void lChangeB();
void rRiseA();
void rChangeB();
void controlLoop();
// Publisher
ros::Publisher sendVelocity("amanita_augusta_mbed/velocity", &sendVelocityMsg);
// Subscriber
ros::Subscriber<geometry_msgs::Twist> receiveVelocity("amanita_augusta_pc/velocity", receiveVelocityFunc);

// Diff Robot
bra::DiffRobot AmanitaAugusta(WHEELS_RADIUS, WHEELS_RADIUS, LENGHT_WHEELS);

// mbed
Ticker mbedRate;
Timer mbedTimer; // 20Hz

int main() {
	AmanitaAugusta.setupMonsterDrivers( A0, L_PWM, L_CW, L_CCW,  R_ENABLE, R_PWM, R_CW, R_CCW);
	AmanitaAugusta.setupEncoders(L_PHASE_A, L_PHASE_B, lRiseA, lChangeB, L_RESOLUTION, 0, R_PHASE_A, R_PHASE_B, rRiseA, rChangeB, R_RESOLUTION, 1);
	AmanitaAugusta.setupController(L_KP, L_KI, L_KD, R_KP, R_KI, R_KD, RATE_MS);

	wait(1);

	node.initNode();
	node.advertise(sendVelocity);
	node.subscribe(receiveVelocity);
	
	wait(1);

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
	float* velocity = AmanitaAugusta.getVelocity();
	sendVelocityMsg.linear.x = *velocity;
	sendVelocityMsg.angular.z = *(velocity+1);
	sendVelocity.publish(&sendVelocityMsg);
}

void receiveVelocityFunc(const geometry_msgs::Twist& msg){
	AmanitaAugusta.setVelocity(msg.linear.x, msg.angular.z);
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