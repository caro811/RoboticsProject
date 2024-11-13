#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <lazo_abierto/TrajectoryFollower.h>
#include <geometry_msgs/PoseStamped.h>

class KinematicPositionController : public TrajectoryFollower
{
  public:

    KinematicPositionController(ros::NodeHandle& nh);

    bool control(const ros::Time& t, double& vx, double& vy, double& w);

  private:

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener transform_listener_;
    ros::Publisher expected_position_pub;

    bool print_trajectory_follow_info;

    double K1, K2, K3;
    double LOOKAHEAD;
    bool EKF;

    unsigned int previous_closest = 0;

  // funciones auxiliares

    bool getCurrentPose(const ros::Time& t, double& x, double& y, double& a);

    bool getCurrentGoal(const ros::Time& t, double& x, double& y, double& a){
      return getPursuitBasedGoal(t, x, y, a);
    }

    bool getPursuitBasedGoal(const ros::Time& t, double& x, double& y, double& a);

    void publishCurrentGoal(const ros::Time& t, const double& goal_x, const double& goal_y, const double& goal_a){
      geometry_msgs::PoseStamped expected_pose;
      expected_pose.header.frame_id = "map";
      expected_pose.header.stamp = t;
      expected_pose.pose.position.x = goal_x;
      expected_pose.pose.position.y = goal_y;
      expected_pose.pose.position.z = 0;
      tf2::Quaternion orientation;
      orientation.setRPY(0,0,goal_a);
      expected_pose.pose.orientation = tf2::toMsg(orientation);
      expected_position_pub.publish(expected_pose);
    }
};

