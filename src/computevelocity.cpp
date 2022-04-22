#include "ros/ros.h"

#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

double y = 0;
double x = 0;

// TODO: to change (seen on GitHub)
enum Wheels {front_left, front_right, rear_left, rear_right};

struct RobotParams {
	double gearRatio, wheelRadius, wheelAlongX, wheelAlongY, encoderResolution;
};

double vxFromWheelSpeeds(std::vector<double> speeds){
    //return -(r/4)*(speeds[front_left]+speeds[front_right]+speeds[rear_left]+speeds[rear_right]);
    return -(0.07/4)*(speeds[front_left]+speeds[front_right]+speeds[rear_left]+speeds[rear_right]);
}

double vyFromWheelSpeeds(std::vector<double> speeds){
    //return (r/4)*(speeds[front_left]-speeds[front_right]-speeds[rear_left]+speeds[rear_right]);
    return (0.07/4)*(speeds[front_left]-speeds[front_right]-speeds[rear_left]+speeds[rear_right]);
}

// TODO: to change (seen on GitHub)
geometry_msgs::TwistStamped velFromEncoders(const sensor_msgs::JointState::ConstPtr& msg) {
	std::vector<double> positions = msg->position;
  	std::vector<double> speeds = msg->velocity;

  	double vx = vxFromWheelSpeeds(speeds);
 	double vy = vyFromWheelSpeeds(speeds);
  	y+=vy;
  	x+=vx;

	geometry_msgs::TwistStamped topub;
	topub.twist.linear.x = x;
    topub.twist.linear.y = y;
    
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "computevelocity");
	ros::NodeHandle n;

	ros::Subscriber sub_wheel_states = n.subscribe("wheel_states", 1000);
	ros::Publisher velocity_update = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

	ros::Rate loop_rate(100);

	while (ros::ok()) {

	    // generate velocity update msg
	    ros::spinOnce();

	    loop_rate.sleep();
	}
	
	return 0;
}