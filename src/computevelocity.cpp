#include "ros/ros.h"

#include "std_msgs/String.h"

#include "math.h"

#include <iostream>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <first_project/wheels_rpm_msg.h>
#include <first_project/reset.h>
#include <nav_msgs/Odometry.h>
#include <chrono>

double x = 0;
double y = 0;
double theta = 0;

ros::Time current_time;
ros::Time old_time;

//time variables used by odometry algorithm
ros::Time current_time_odom;
ros::Time old_time_odom;

//publishes messages that has robot's velocities informations
ros::Publisher velocity_update;

//subscribes to velocity_update to read robot's velocities messages to calcilate wheels speeds
ros::Subscriber sub_velocity_update_for_wheels_speeds;

//subscribes to velocity_update to read robot's velocities messages to apply euler algorithm in order to get the new pose
ros::Subscriber sub_velocity_update_for_Odometry;

//publishes messages that contains new robot's pose information get using euler algorithm
ros::Publisher euler_odometry;


ros::Publisher wheels_rpm;

enum WheelsPosition {fl, fr, rl, rr};

struct RobotParams {
	int gearRatio, encoderResolution;
	double wheelRadius, wheelAlongX, wheelAlongY;
};


RobotParams RobParams;

bool reset_callback(first_project::reset::Request  &req, 
                    first_project::reset::Response &res) {

	//TODO (optional): print old position
	x = req.new_x;
	y = req.new_y;
	theta = req.new_ang;

	ROS_INFO("Position reset");
	
	return true;
}

void speedFromEncoders(const sensor_msgs::JointState::ConstPtr& msg) {

	float wheelSpeeds[3];
	float wheelDeltaTicks[3];

	std::vector<double> robotOdometry(0,0.0);

	float oldTicks[3];

	ros::Time current_time = msg->header.stamp; 
	float ticks_dt = (current_time - old_time).toSec();
	// ticks_dtm is 1s, because speedFromEncoders is a callback triggered every 1s 
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

//callback function that reads message from cmd_vel topic and uses it to calculate wheel speeds
void wheel_speeds_from_cmd_vel(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	//saving three velocities of the robot from cmd_vel topic
	double robot_linear_velocity_x = msg->twist.linear.x;
	double robot_linear_velocity_y = msg->twist.linear.y;
	double robot_angular_velocity_z = msg->twist.linear.y;

	//calculating wheels' velocities
	//wheel 1
	float fl_wheel_velocity = ( 1 / RobParams.wheelRadius ) * ((- RobParams.wheelAlongX - RobParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x - robot_linear_velocity_y);
	//wheel 2
	float fr_wheel_velocity = (1/RobParams.wheelRadius) * ((RobParams.wheelAlongX + RobParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x + robot_linear_velocity_y);
	//wheel 3
	float rr_wheel_velocity = (1/RobParams.wheelRadius) * ((RobParams.wheelAlongX + RobParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x - robot_linear_velocity_y);
	//wheel 4
	float rl_wheel_velocity = (1/RobParams.wheelRadius) * ((- RobParams.wheelAlongX - RobParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x + robot_linear_velocity_y);

	//creating the message that will be published in wheels_rpm topic
	first_project::wheels_rpm_msg msg_to_publish;
	msg_to_publish.header.stamp.sec = current_time.toSec();
	msg_to_publish.rpm_fl = fl_wheel_velocity;
	msg_to_publish.rpm_fr = fr_wheel_velocity;
	msg_to_publish.rpm_rr = rr_wheel_velocity;
	msg_to_publish.rpm_rl = rr_wheel_velocity;

	//publishing the message
	wheels_rpm.publish(msg_to_publish);
}

//using messages read from topic cmd_vel calculates the odometry of the robot
void euler_odometry_algorithm(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	current_time_odom = msg->header.stamp;
	float dt = (current_time_odom - old_time_odom).toSec();

	//velocities of the robot
	float vx = msg->twist.linear.x;
	float vy = msg->twist.linear.y;
	float omega = msg->twist.angular.z;
	float v = sqrt(pow(vx, 2) + pow(vy, 2));

	//applying euler algoritm for the odometry 
	float x_after_dt = x + v * dt * cos(theta);
	float y_after_dt = y + v * dt * sin(theta);
	float theta_after_dt = theta + omega * dt;

	//creating and publishing the message that contains informations of new pose of the robot
	nav_msgs::Odometry msg_to_publish;
	msg_to_publish.header.stamp.sec = current_time.toSec();
	msg_to_publish.pose.pose.position.x = x_after_dt;
	msg_to_publish.pose.pose.position.y = y_after_dt;
	msg_to_publish.pose.pose.orientation.z = theta_after_dt;

	//publishing the message
	euler_odometry.publish(msg_to_publish);

	//updating variables for next use
	x = x_after_dt;
	y = y_after_dt;
	theta = theta_after_dt;
	old_time_odom = current_time_odom;
}

/*
void runge_kutta_algorithm(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	current_time_odom = msg->header.stamp;
	float dt = (current_time_odom - old_time_odom).toSec();

	float vx = msg->twist.linear.x;
	float vy = msg->twist.linear.y;
	float omega = msg->twist.angular.z;

	float v = sqrt(pow(vx, 2) + pow(vy, 2));

	float x_after_dt = x + v * dt * cos(theta + (omega * dt)/2);
	float y_after_dt = y + v * dt * sin(theta + (omega * dt)/2);
	float theta_after_dt = theta + omega * dt;

	nav_msgs/Odometry msg_to_publish;
	msg_to_publish.header.stamp.sec = current_time.toSec();
	msg_to_publish.pose.pose.position.x = x_after_dt;
	msg_to_publish.pose.pose.position.y = y_after_dt;
	msg_to_publish.pose.pose.orientation.z = theta_after_dt;

	euler_odometry.publish(msg_to_publish);

	x = x_after_dt;
	y = y_after_dt;
	theta = theta_after_dt;

	old_time_odom = current_time_odom;
}
*/

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

	//subscribing to cmd_vel topic, where the velocity_update publishes. Callback function will calculate wheels velocities
	ros::Subscriber sub_velocity_update_for_wheels_speeds = n.subscribe("cmd_vel", 1000, wheel_speeds_from_cmd_vel);
	
	//calculated velocities are published in wheel_rpm topic
	wheels_rpm = n.advertise<first_project::wheels_rpm_msg>("wheel_rpm", 1000);

	//subscribing to cmd_vel topic, where the velocity_update publishes. Callback function will calculate new robot's pose using euler algorithm
	ros::Subscriber sub_velocity_update_for_Odometry = n.subscribe("cmd_vel", 1000, euler_odometry_algorithm);

	//after calculating new poses with euler algorithm, ther are published in odom topic
	euler_odometry = n.advertise<nav_msgs::Odometry>("odom", 1000);

	//Define reset service handler
  	ros::ServiceServer service = n.advertiseService<first_project::reset::Request, 
                         first_project::reset::Response>("Reset", reset_callback);

	ros::Rate loop_rate(100);

	while (ros::ok()) {

	    ros::spinOnce();
	    loop_rate.sleep();
	
	}
	
	
	return 0;
}