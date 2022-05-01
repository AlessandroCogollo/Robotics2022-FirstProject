#include "ros/ros.h"

#include "std_msgs/String.h"

#include "math.h"

#include <iostream>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <chrono>

double y = 0;
double x = 0;

ros::Time current_time;
ros::Time old_time;

ros::Publisher velocity_update;

enum WheelsPosition {fl, fr, rl, rr};

struct RobotParams {
	int gearRatio, encoderResolution;
	double wheelRadius, wheelAlongX, wheelAlongY;
};

RobotParams RobParams;

void speedFromEncoders(const sensor_msgs::JointState::ConstPtr& msg) {

	float wheelSpeeds[3];
	float wheelDeltaTicks[3];

	std::vector<double> robotOdometry(0,0.0);

	float oldTicks[3];

	ros::Time current_time = msg->header.stamp; 
	float ticks_dt = (current_time - old_time).toSec();
	float ticks_dtm = ticks_dt / 60;

	for (int i = 0; i < 4; i++) {
		
		wheelDeltaTicks[i] = msg->position[i] - oldTicks[i];
		wheelSpeeds[i] = (wheelDeltaTicks[i] * 2 * M_PI) / (ticks_dtm * RobParams.gearRatio * RobParams.encoderResolution);
		
		oldTicks[i] = msg->position[i];
	}

	// omega1 = wheel1 = fl
	// omega2 = wheel2 = fr
	// omega3 = wheel4 = rl
	// omega4 = wheel3 = rr

	// vx = (omega1 + omega2 + omega3 + omega4) * r/4
	double vx = (wheelSpeeds[fl] + wheelSpeeds[fr] + wheelSpeeds[rl] + wheelSpeeds[rr]) * RobParams.wheelRadius/4;
	// vy = (-omega1 + omega2 + omega3 - omega4) * r/4
	double vy = (- wheelSpeeds[fl] + wheelSpeeds[fr] + wheelSpeeds[rl] - wheelSpeeds[rr]) * RobParams.wheelRadius/4;
	// angular = (-omega1 + omega2 - omega3 + omega4) * r/4(lx + ly)
	double angular = (- wheelSpeeds[fl] + wheelSpeeds[fr] - wheelSpeeds[rl] + wheelSpeeds[rr]) * RobParams.wheelRadius/ (4 * (RobParams.wheelAlongX + RobParams.wheelAlongY));

	geometry_msgs::TwistStamped msg_to_publish;
	msg_to_publish.header.stamp.sec = current_time.toSec();
	msg_to_publish.twist.linear.x  = vx;
	msg_to_publish.twist.linear.y  = vy;
	msg_to_publish.twist.angular.z = angular;
	velocity_update.publish(msg_to_publish);
	
	old_time = current_time;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "computevelocity");
	ros::NodeHandle n;

	// retrieve RobotParameters from up.launch
	ros::param::get("/gearRatio", RobParams.gearRatio);
	ros::param::get("/wheelRadius", RobParams.wheelRadius);
	ros::param::get("/wheelAlongX", RobParams.wheelAlongX);
	ros::param::get("/wheelAlongY", RobParams.wheelAlongY);
	ros::param::get("/encoderResolution", RobParams.encoderResolution);

	velocity_update = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
	ros::Subscriber sub_wheel_states = n.subscribe("wheel_states", 1000, speedFromEncoders);

	ros::Rate loop_rate(100);

	while (ros::ok()) {

	    ros::spinOnce();
	    loop_rate.sleep();
	
	}
	
	return 0;
}