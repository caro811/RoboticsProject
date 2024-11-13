#ifndef __ROBMOVIL_EKF_LOCALIZER_H__
#define __ROBMOVIL_EKF_LOCALIZER_H__

#include <ros/ros.h>
#include <robmovil_msgs/LandmarkArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include "localizer_ekf.h"

namespace robmovil_ekf {
  class Localizer
  {
    public:
      Localizer(ros::NodeHandle& n);

      void get_posts(const geometry_msgs::PoseArrayConstPtr& posts);
      void on_landmark_array(const robmovil_msgs::LandmarkArrayConstPtr& msg);
      void on_odometry(const nav_msgs::OdometryConstPtr& msg);

    private:
      LocalizerEKF ekf;

      bool set_map;

      bool print_localizer_info;

      bool only_prediction;

      ros::Subscriber landmark_sub, odo_sub, posts_sub;
      ros::Publisher pose_pub;
      std::string base_frame_, map_frame_, laser_frame_;

      ros::Timer prediction_timer;
      tf2_ros::TransformBroadcaster transform_broadcaster_;

      void prediction_event(const ros::TimerEvent& event);
      void advance_time(const ros::Time& now);
      void publish_estimate(const ros::Time& now);

      void prediction(const ros::Time& time);

      /* ultimo comando de control recibido */
      LocalizerEKF::Vector u;
      ros::Time t;
  };
}

#endif // __ROBMOVIL_EKF_LOCALIZER_H__
