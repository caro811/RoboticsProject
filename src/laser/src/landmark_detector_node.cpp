#include <ros/ros.h>
#include "landmark_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "landmark_detector");
  ros::NodeHandle n("~");
  robmovil_ekf::LandmarkDetector landmark_detector(n);
  ros::spin();
  return 0;
}
