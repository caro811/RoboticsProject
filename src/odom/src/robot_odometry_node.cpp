#include <ros/ros.h>
#include "robot_odometry.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_odometry");
  ros::NodeHandle nh;
  robmovil::RobotOdometry robot_odometry(nh);
  ros::spin();
  return 0;
}
