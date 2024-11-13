#include <ros/ros.h>
#include "KinematicPositionController.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_follower");
  ros::NodeHandle nh;

  KinematicPositionController trajectory_follower( nh );

  ros::spin();
  
  trajectory_follower.stop_timer();

  return 0;
}
