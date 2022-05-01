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
ros::Subscriber sub_velocity_update;

ros::Publisher wheels_rpm;

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

void wheel_speeds_from_cmd_vel(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	//saving three velocities of the robot from cmd_vel topic
	double robot_linear_velocity_x = msg.twist.linear.x;
	double robot_linear_velocity_y = msg.twist.linear.y;
	double robot_angular_velocity_z = msg.twist.angular.z;

	//calculating wheels' velocities
	//wheel 1
	float fl_wheel_velocity = (1/RobotParams.wheelRadius) * ((- RobotParams.wheelAlongX - RobotParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x - robot_linear_velocity_y);
	//wheel 2
	float fr_wheel_velocity = (1/RobotParams.wheelRadius) * ((RobotParams.wheelAlongX + RobotParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x + robot_linear_velocity_y);
	//wheel 3
	float rr_wheel_velocity = (1/RobotParams.wheelRadius) * ((RobotParams.wheelAlongX + RobotParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x - robot_linear_velocity_y);
	//wheel 4
	float rl_wheel_velocity = (1/RobotParams.wheelRadius) * ((- RobotParams.wheelAlongX - RobotParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x + robot_linear_velocity_y);

	//creating the message that will be published in wheels_rpm topic
	wheels_rpm_msg msg_to_publish;
	msg_to_publish.header.stamp.sec = current_time.toSec();
	msg_to_publish.rmp_fl = fl_wheel_velocity;
	msg_to_publish.rmp_fr = fr_wheel_velocity;
	msg_to_publish.rmp_rr = rr_wheel_velocity;
	msg_to_publish.rmp_rl = rr_wheel_velocity;

	//publishing the message
	wheels_rpm.publish(msg_to_publish);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "computevelocity");
	ros::NodeHandle n;

	// retrieve RobotParameters from up.launch
	ros::param::get("/gearRatio", RobParams.gearRatio);
	ros::param::get("/wheelRadius", RobParams.wheelRadius);
	ros::param::get("/wheelAlongX", RobParams.wheelAlongX);
	ros::param::get("/wheelAlongY", RobParams.wheelAlongY);
	ros::param::get("/encoderResolution", RobParams.encoderResolution);

	//subscribing to wheel_states topic, created from the bag. Callback function will calculate robot's linear velocities along x and y and angular velocities along z 
	ros::Subscriber sub_wheel_states = n.subscribe("wheel_states", 1000, speedFromEncoders);

	velocity_update = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

	//subscribing to cmd_vel topic, where the velocity_update publishes. Callback function will calculate wheels velocities
	ros::Subscriber sub_velocity_update = n.subscribe("cmd_vel", 1000, wheel_speeds_from_cmd_vel);
	//calculated velocities are published in wheel_rpm topic
	wheels_rpm = n.advertise<wheels_rpm_msg>("wheel_rpm", 1000)

	ros::Rate loop_rate(100);

	while (ros::ok()) {

	    ros::spinOnce();
	    loop_rate.sleep();
	
	}
	
	return 0;
}