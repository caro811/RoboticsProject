#include <ros/ros.h>
#include "localizer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localizer");
  ros::NodeHandle n("~");
  robmovil_ekf::Localizer localizer(n);
  ros::spin();
  return 0;
}
