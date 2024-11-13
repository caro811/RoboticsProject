#include <ros/ros.h>
#include <lazo_abierto/FeedForwardController.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_follower");
  ros::NodeHandle nh;

  FeedForwardController trajectory_follower( nh );

  ros::spin();
  
  trajectory_follower.stop_timer();

  return 0;
}
