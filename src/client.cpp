#include "ros/ros.h"
#include "pub_sub/Reset.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_client");
  if (argc != 2)
  {
    ROS_INFO("usage: reset odometry");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<first_project::Reset>("reset");

  first_project::Reset srv;
  srv.request.new_count = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Old position: \\
      x %lf \\
      y %lf \\
      theta %lf",
      srv.response.old_count);
  }
  else
  {
    ROS_ERROR("Failed to call service reset");
    return 1;
  }

  return 0;
}
