#ifndef __ROBMOVIL_EKF_LANDMARK_DETECTOR_H__
#define __ROBMOVIL_EKF_LANDMARK_DETECTOR_H__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <robmovil_msgs/LandmarkArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace robmovil_ekf
{
  /*
   * Calcula landmarks representados en coordenadas polares, en el sistema de referencia del robot
   */

  class LandmarkDetector
  {
    public:
      LandmarkDetector(ros::NodeHandle& n);

      void on_laser_scan(const sensor_msgs::LaserScanConstPtr& msg);

    private:
      ros::NodeHandle& n;
      void publish_pointcloud(const std_msgs::Header& header, const std::vector<tf::Vector3>& landmark_positions);
      bool update_laser_tf(const ros::Time& required_time);

      ros::Subscriber laser_sub;
      ros::Publisher landmark_pub, pointcloud_pub;

      std::string robot_frame, laser_frame, publish_robot_frame;

      boost::shared_ptr<tf::TransformListener> listener;
      tf::StampedTransform laser_transform;
      bool transform_received;

      bool print_laser_info;
  };
}

#endif // __ROBMOVIL_EKF_LANDMARK_DETECTOR_H__
