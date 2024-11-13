#include <fstream>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lazo_cerrado/tf_utils.hpp>
#include <nav_msgs/Odometry.h>

/*************************************************************************************************/
// Clase
class LoggerOdom{
    public:

        LoggerOdom(ros::NodeHandle& nh);

    private:

        ros::Subscriber robot_pose_sub_, ground_truth_sub_;

        std::ofstream robot_logfile_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener transform_listener_;

        void handleRobotPose(const nav_msgs::Odometry& msg);
};

/*************************************************************************************************/
// Constructor
LoggerOdom::LoggerOdom(ros::NodeHandle& nh)
    : robot_logfile_("Odom_poses.log"), transform_listener_( tfBuffer_ )
{
    robot_pose_sub_ = nh.subscribe("/robot/odometry", 1, &LoggerOdom::handleRobotPose, this);
}

/*************************************************************************************************/
// Publicar poses
void LoggerOdom::handleRobotPose(const nav_msgs::Odometry& msg){
    tf2::Transform gt;
    while(not lookupTransformSafe(tfBuffer_, "odom", "base_link_gt", ros::Time::now(), gt)) ;

    robot_logfile_ << msg.header.stamp.toSec() << std::endl;
    robot_logfile_ << " - Odometry:     " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw(msg.pose.pose.orientation) << std::endl;
    robot_logfile_ << " - Ground truth: " << gt.getOrigin().getX() << " " << gt.getOrigin().getY() << " " << tf2::getYaw(gt.getRotation()) << std::endl;
}

/*************************************************************************************************/
// Main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "loggerOdom");
    ros::NodeHandle nh;

    LoggerOdom LoggerOdom(nh);

    ros::spin();

    return 0;
}

