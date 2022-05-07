#include "math.h"
#include "ros/ros.h"

#include <chrono>
#include <iostream>
#include <ros/console.h>

#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <first_project/ParametersConfig.h>
#include <first_project/wheels_rpm_msg.h>
#include <first_project/reset.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>

bool firstTime = true;

// Euler (default, 0), RK (1)
int integrationMethod = 0;

// 1.2.b Ros Parameter For Initial Pose
double x = 0;
double y = 0;
double theta = 0;

double maxPoseError = 0.1;
double maxCalibrationChange = 0.5;

ros::Time current_time;
ros::Time old_time;

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

// STEP 4
void IMreconfigure(int* integrationMethod, first_project::ParametersConfig &config) {
	*integrationMethod = config.integration_method;

	RobotParams oldParams = RobParams;

	RobParams.gearRatio = config.gear_ratio;
	RobParams.wheelRadius = config.wheel_radius;
	RobParams.wheelAlongX = config.wheel_along_x;
	RobParams.wheelAlongY = config.wheel_along_y;
	RobParams.encoderResolution = config.encoder_resolution;

	ROS_INFO("GearRatio: Old: %d -> New: %d", oldParams.gearRatio, RobParams.gearRatio);
	ROS_INFO("WheelRadius: Old: %f -> New: %f", oldParams.wheelRadius, RobParams.wheelRadius);
	ROS_INFO("WheelAlongX: Old: %f -> New: %f", oldParams.wheelAlongX, RobParams.wheelAlongX);
	ROS_INFO("WheelAlongY: Old: %f -> New: %f", oldParams.wheelAlongY, RobParams.wheelAlongY);
	ROS_INFO("EncoderResolution: Old: %d -> New: %d", oldParams.encoderResolution, RobParams.encoderResolution);
    ROS_INFO("Reconfigure request: %d", *integrationMethod);

}

// STEP 3
bool reset_callback(first_project::reset::Request  &req, 
                    first_project::reset::Response &res) {

	//TODO (optional): print old position
	x = req.new_x;
	y = req.new_y;
	theta = req.new_ang;

	ROS_INFO("Position reset");
	
	return true;
}

void computeOdometry(const sensor_msgs::JointState::ConstPtr& msg) {
	
	double wheelSpeeds[3];
	double wheelDeltaTicks[3];

	std::vector<double> robotOdometry(0,0.0);

	double oldTicks[3];

	ros::Time current_time = msg->header.stamp; 
	double ticks_dt = (current_time - old_time).toSec();

	if (firstTime) {

		firstTime = false;
		old_time = current_time;
		for (int i = 0; i < 4; i++) {
			oldTicks[i] = msg->position[i];
		}

	} else {

		for (int i = 0; i < 4; i++) {

			// STEP 1.1.b
			
			wheelDeltaTicks[i] = msg->position[i] - oldTicks[i];
			wheelSpeeds[i] = (wheelDeltaTicks[i] * 2 * M_PI) / (ticks_dt * RobParams.gearRatio * RobParams.encoderResolution);
			
			oldTicks[i] = msg->position[i];
		}

		// STEP 1.1.c (and STEP 1.1.a)
		// omega1 = wheel1 = fl | omega2 = wheel2 = fr | omega3 = wheel4 = rl | omega4 = wheel3 = rr

		// vx = (omega1 + omega2 + omega3 + omega4) * r/4
		double vx = (wheelSpeeds[fl] + wheelSpeeds[fr] + wheelSpeeds[rl] + wheelSpeeds[rr]) * RobParams.wheelRadius/4;
		// vy = (-omega1 + omega2 + omega3 - omega4) * r/4
		double vy = (- wheelSpeeds[fl] + wheelSpeeds[fr] + wheelSpeeds[rl] - wheelSpeeds[rr]) * RobParams.wheelRadius/4;
		// angular = (-omega1 + omega2 - omega3 + omega4) * r/4(lx + ly)
		double angular = (- wheelSpeeds[fl] + wheelSpeeds[fr] - wheelSpeeds[rl] + wheelSpeeds[rr]) * RobParams.wheelRadius/ (4 * (RobParams.wheelAlongX + RobParams.wheelAlongY));

		// STEP 1.1.d
		geometry_msgs::TwistStamped TSmsg;
		TSmsg.header.stamp.sec = current_time.toSec();
		TSmsg.twist.linear.x  = vx;
		TSmsg.twist.linear.y  = vy;
		TSmsg.twist.angular.z = angular;
		velocity_update.publish(TSmsg);

		// STEP 1.2.a
		double v = sqrt(pow(vx, 2) + pow(vy, 2));

		double x_after_dt, y_after_dt, theta_after_dt;

		switch(integrationMethod) {
			case 0: {
				//applying euler algoritm for the odometry 
				x_after_dt = x + v * ticks_dt * cos(theta);
				y_after_dt = y + v * ticks_dt * sin(theta);
				theta_after_dt = theta + angular * ticks_dt;
			} break;
			case 1: {
				//applying runge-kutta algorithm for the odometry
				x_after_dt = x + v * ticks_dt * cos(theta + (angular * ticks_dt)/2);
				y_after_dt = y + v * ticks_dt * sin(theta + (angular * ticks_dt)/2);
				theta_after_dt = theta + angular * ticks_dt;
			} break;
			default: {
				//applying euler algoritm for the odometry 
				x_after_dt = x + v * ticks_dt * cos(theta);
				y_after_dt = y + v * ticks_dt * sin(theta);
				theta_after_dt = theta + angular * ticks_dt;
			} break;
		}

		// STEP 1.2.c creating and publishing the message that contains informations of new pose of the robot
		nav_msgs::Odometry ODmsg;
		ODmsg.header.stamp.sec = current_time.toSec();
		ODmsg.pose.pose.position.x = x_after_dt;
		ODmsg.pose.pose.position.y = y_after_dt;
		ODmsg.pose.pose.orientation.z = theta_after_dt;
		euler_odometry.publish(ODmsg);

		// STEP 1.2.d
		static tf2_ros::TransformBroadcaster tf2_broadcaster;
	    geometry_msgs::TransformStamped transformStamped;
	    transformStamped.header.stamp = current_time;
	    transformStamped.header.frame_id = "world";
	    transformStamped.child_frame_id = "robot";
	    transformStamped.transform.translation.x = ODmsg.pose.pose.position.x;
	    transformStamped.transform.translation.y = ODmsg.pose.pose.position.y;
	    transformStamped.transform.translation.z = ODmsg.pose.pose.orientation.z;
	    tf2::Quaternion q;
	    q.setRPY(0, 0, ODmsg.pose.pose.orientation.z);
	    transformStamped.transform.rotation.x = q.x();
	    transformStamped.transform.rotation.y = q.y();
	    transformStamped.transform.rotation.z = q.z();
	    transformStamped.transform.rotation.w = q.w(); 
	 	tf2_broadcaster.sendTransform(transformStamped);

		//updating variables for next use
		x = x_after_dt;
		y = y_after_dt;
		theta = theta_after_dt;
		old_time = current_time;
	}
};

void computeControl(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	
	//saving three velocities of the robot from cmd_vel topic
	double robot_linear_velocity_x = msg->twist.linear.x;
	double robot_linear_velocity_y = msg->twist.linear.y;
	double robot_angular_velocity_z = msg->twist.angular.z;

	// STEP 2.1 calculating wheels' velocities
	//wheel 1
	double fl_wheel_velocity = (1/RobParams.wheelRadius) * ((- RobParams.wheelAlongX - RobParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x - robot_linear_velocity_y);
	//wheel 2
	double fr_wheel_velocity = (1/RobParams.wheelRadius) * ((RobParams.wheelAlongX + RobParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x + robot_linear_velocity_y);
	//wheel 3
	double rr_wheel_velocity = (1/RobParams.wheelRadius) * ((RobParams.wheelAlongX + RobParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x - robot_linear_velocity_y);
	//wheel 4
	double rl_wheel_velocity = (1/RobParams.wheelRadius) * ((- RobParams.wheelAlongX - RobParams.wheelAlongY) * robot_angular_velocity_z + robot_linear_velocity_x + robot_linear_velocity_y);

	double rpm_fl = (fl_wheel_velocity * RobotParams.gearRatio * 60)/(2*M_PI);
	double rpm_fr = (fr_wheel_velocity * RobotParams.gearRatio * 60)/(2*M_PI);
	double rpm_rr = (rr_wheel_velocity * RobotParams.gearRatio * 60)/(2*M_PI);
	double rpm_rl = (rl_wheel_velocity * RobotParams.gearRatio * 60)/(2*M_PI);

	// STEP 2.2 (and STEP 2.3) creating the message that will be published in wheels_rpm topic
	first_project::wheels_rpm_msg msg_to_publish;
	msg_to_publish.header.stamp.sec = current_time.toSec();
	msg_to_publish.rpm_fl = rpm_fl;
	msg_to_publish.rpm_fr = rpm_fr;
	msg_to_publish.rpm_rr = rpm_rr;
	msg_to_publish.rpm_rl = rpm_rl;

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

	//dynamic_reconfigure for integrationMethod
	dynamic_reconfigure::Server<first_project::ParametersConfig> IMserver;
    IMserver.setCallback(boost::bind(&IMreconfigure, &integrationMethod, _1));

    euler_odometry = n.advertise<nav_msgs::Odometry>("odom", 1000);
	velocity_update = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
	ros::Subscriber sub_wheel_states = n.subscribe("wheel_states", 1000, computeOdometry);

	//subscribing to cmd_vel topic, where the velocity_update publishes. Callback function will calculate wheels velocities
	wheels_rpm = n.advertise<first_project::wheels_rpm_msg>("wheel_rpm", 1000);
	ros::Subscriber sub_velocity_update_for_wheels_speeds = n.subscribe("cmd_vel", 1000, computeControl);

	ros::Subscriber sub_robot_pose = n.advertise<geometry_msgs::PoseStamped>("/robot/pose", 1000, parametersCalibration)

	//defining reset service handler
  	ros::ServiceServer service = n.advertiseService<first_project::reset::Request, 
                         first_project::reset::Response>("Reset", reset_callback);

	ros::Rate loop_rate(100);

	while (ros::ok()) {

	    ros::spinOnce();
	    loop_rate.sleep();
	
	}
	
	
	return 0;
}