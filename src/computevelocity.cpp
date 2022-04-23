#include "ros/ros.h"

#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

double y = 0;
double x = 0;

std::vector<double> speeds;
std::vector<double> OldPosition{ 0, 0, 0, 0};
std::vector<double> NewPosition;

enum WheelsPosition {fl, fr, rl, rr};

struct RobotParams {
	int gearRatio, encoderResolution;
	double wheelRadius, wheelAlongX, wheelAlongY;
};

void speedFromEncoders(const sensor_msgs::JointState::ConstPtr& msg) {
	float vfl, vfr, vrl, vrr;
	for (int i = 0; i < 4; i++) {
		 NewPosition.push_back(msg->position[i]);
	}
	OldPosition = NewPosition;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "computevelocity");
	ros::NodeHandle n;

	// retrieve RobotParameters from up.launch
	RobotParams RobParams;
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