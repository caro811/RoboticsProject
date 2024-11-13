#include <ros/ros.h>

#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>

#define HAVE_VREP_COMMON 0 /* TODO: define this from the cmakelists */

#if HAVE_VREP_COMMON
#include <vrep_common/VrepInfo.h>
#endif

ros::Publisher clock_publisher;
ros::Subscriber vrep_subscriber;

ros::Time start_time;
float last_vrep_time;

void reset_start_time(void)
{
	ros::WallTime wall_time = ros::WallTime::now();
  start_time = ros::Time(wall_time.sec, wall_time.nsec);
	last_vrep_time = -1;
}

#if HAVE_VREP_COMMON
void on_vrep_info(const vrep_common::VrepInfo::ConstPtr& vrep_info)
{
	float new_vrep_time = vrep_info->simulationTime.data;

	if (!(vrep_info->simulatorState.data & 1)) /* if simulator is stopped, reset time offset */
	{
		reset_start_time();
	}
	else if (last_vrep_time != new_vrep_time)
	{
		last_vrep_time = new_vrep_time;

		rosgraph_msgs::Clock c;
		c.clock = start_time + ros::Duration(new_vrep_time);
		clock_publisher.publish(c);
	}
}
#endif

void on_vrep_time(const std_msgs::Float64 msg)
{
  ROS_DEBUG("vrep time");
  rosgraph_msgs::Clock c;
  c.clock = ros::Time(msg.data);
  clock_publisher.publish(c);
}

int main(int argc, char** argv)
{
   ros::init(argc,argv, "clock_publisher");

	 reset_start_time();

   ros::NodeHandle node_handle;

   bool ros_interface = true;
   node_handle.param("ros_interface", ros_interface, ros_interface);
   if (ros_interface)
     ROS_INFO("Using RosInterface");
   else
#if HAVE_VREP_COMMON
     ROS_INFO("Using RosPlugin");
#else
   {
     ROS_ERROR("RosPlugin support not compiled!");
     return 1;
   }
#endif
   
   if (ros_interface)
     vrep_subscriber = node_handle.subscribe("/vrep_time", 1, on_vrep_time);
#if HAVE_VREP_COMMON
   else
     vrep_subscriber = node_handle.subscribe("/vrep/info", 1, on_vrep_info);
#endif

   clock_publisher = node_handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

   ros::spin();
}
