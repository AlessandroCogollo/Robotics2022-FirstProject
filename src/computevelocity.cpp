#include "ros/ros.h"

#include "std_msgs/String.h"

#include <iostream>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <chrono>

double y = 0;
double x = 0;

ros::Time current_time;
ros::Time old_time;


enum WheelsPosition {fl, fr, rl, rr};

struct RobotParams {
	int gearRatio, encoderResolution;
	double wheelRadius, wheelAlongX, wheelAlongY;
};

RobotParams RobParams;

void speedFromEncoders(const sensor_msgs::JointState::ConstPtr& msg) {

	float wheelSpeeds[3];
	float wheelDeltaTicks[3];
	float oldTicks[3];

	ros::Time current_time = msg->header.stamp; 
	float ticks_dt = (current_time - old_time).toSec();
	float ticks_dtm = ticks_dt / 60;

	for (int i = 0; i < 4; i++) {
		
		wheelDeltaTicks[i] = msg->position[i] - oldTicks[i];
		wheelSpeeds[i] = (wheelDeltaTicks[i] / RobParams.encoderResolution) / ticks_dtm;
		
		// added for debug purposes
		std::cout << std::setprecision(10) << wheelSpeeds[i] << std::endl;		
		
		oldTicks[i] = msg->position[i];
	}
	
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

	ROS_INFO("gearRatio: %d, wheelRadius: %lf, wheelAlongX: %lf", RobParams.gearRatio, RobParams.wheelRadius, RobParams.wheelAlongX);

	ros::Subscriber sub_wheel_states = n.subscribe("wheel_states", 1000, speedFromEncoders);
	ros::Publisher velocity_update = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);	

	ros::Rate loop_rate(100);

	while (ros::ok()) {

	    ros::spinOnce();

	    loop_rate.sleep();
	}
	
	return 0;
}