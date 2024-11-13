#include <fstream>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <lazo_cerrado/tf_utils.hpp>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <tf/tf.h>

/*************************************************************************************************/
class LoggerEKF{
    public:

        LoggerEKF(ros::NodeHandle& nh);

    private:

        ros::Subscriber robot_pose_sub_;

        std::ofstream robot_logfile_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener transform_listener_;

        ros::Publisher odom_path_publisher, gt_path_publisher, ekf_path_publisher;
        nav_msgs::Path odom_path_msg, gt_path_msg, ekf_path_msg;

        void handleRobotPose(const nav_msgs::Odometry& msg);
};

/*************************************************************************************************/
LoggerEKF::LoggerEKF(ros::NodeHandle& nh)
    : robot_logfile_("EKF_poses.log"), transform_listener_( tfBuffer_ )
{
    // Paths descriptos en poses para visualizacion en RViz
    odom_path_publisher = nh.advertise<nav_msgs::Path>("/base_link_path", 1, true);
    gt_path_publisher = nh.advertise<nav_msgs::Path>("/base_link_gt_path", 1, true);
    ekf_path_publisher = nh.advertise<nav_msgs::Path>("/base_link_ekf_path", 1, true);

    robot_pose_sub_ = nh.subscribe("/robot/odometry", 1, &LoggerEKF::handleRobotPose, this);
}

/*************************************************************************************************/
void LoggerEKF::handleRobotPose(const nav_msgs::Odometry& msg){
    /*****************************************************/
    // Publicar odom en rviz
    odom_path_msg.header.stamp = ros::Time::now();
    odom_path_msg.header.frame_id = "odom";

    geometry_msgs::PoseStamped stamped_odom_pose_msg;
    stamped_odom_pose_msg.header.stamp = odom_path_msg.header.stamp;
    stamped_odom_pose_msg.header.frame_id = odom_path_msg.header.frame_id;
    stamped_odom_pose_msg.pose.position.x = msg.pose.pose.position.x;
    stamped_odom_pose_msg.pose.position.y = msg.pose.pose.position.y;
    stamped_odom_pose_msg.pose.position.z = 0;
    stamped_odom_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(tf2::getYaw(msg.pose.pose.orientation));

    odom_path_msg.poses.push_back(stamped_odom_pose_msg);

    odom_path_publisher.publish(odom_path_msg);

    /*****************************************************/
    // Obtener transformacion odom -> base_link_gt
    tf2::Transform gt;
    while(not lookupTransformSafe(tfBuffer_, "odom", "base_link_gt", ros::Time::now(), gt)) ;

    // Publicar gt en rviz
    gt_path_msg.header.stamp = ros::Time::now();
    gt_path_msg.header.frame_id = "odom";

    geometry_msgs::PoseStamped stamped_gt_pose_msg;
    stamped_gt_pose_msg.header.stamp = gt_path_msg.header.stamp;
    stamped_gt_pose_msg.header.frame_id = gt_path_msg.header.frame_id;
    stamped_gt_pose_msg.pose.position.x = gt.getOrigin().getX();
    stamped_gt_pose_msg.pose.position.y = gt.getOrigin().getY();
    stamped_gt_pose_msg.pose.position.z = 0;
    stamped_gt_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(tf2::getYaw(gt.getRotation()));

    gt_path_msg.poses.push_back(stamped_gt_pose_msg);

    gt_path_publisher.publish(gt_path_msg);

    /*****************************************************/
    // Obtener transformacion map -> base_link_ekf
    tf2::Transform ekf;
    while(not lookupTransformSafe(tfBuffer_, "map", "base_link_ekf", ros::Time::now(), ekf)) ;

    // Publicar ekf en rviz
    ekf_path_msg.header.stamp = ros::Time::now();
    ekf_path_msg.header.frame_id = "map";

    geometry_msgs::PoseStamped stamped_ekf_pose_msg;
    stamped_ekf_pose_msg.header.stamp = ekf_path_msg.header.stamp;
    stamped_ekf_pose_msg.header.frame_id = ekf_path_msg.header.frame_id;
    stamped_ekf_pose_msg.pose.position.x = ekf.getOrigin().getX();
    stamped_ekf_pose_msg.pose.position.y = ekf.getOrigin().getY();
    stamped_ekf_pose_msg.pose.position.z = 0;
    stamped_ekf_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(tf2::getYaw(ekf.getRotation()));

    ekf_path_msg.poses.push_back(stamped_ekf_pose_msg);

    ekf_path_publisher.publish(ekf_path_msg);

    /*****************************************************/
    // Publicar en el log
    robot_logfile_ << msg.header.stamp.toSec() << std::endl;
    robot_logfile_ << " - Odometry:     " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw(msg.pose.pose.orientation) << std::endl;
    robot_logfile_ << " - Ground truth: " << gt.getOrigin().getX() << " " << gt.getOrigin().getY() << " " << tf2::getYaw(gt.getRotation()) << std::endl;
    robot_logfile_ << " - EKF: " << ekf.getOrigin().getX() << " " << ekf.getOrigin().getY() << " " << tf2::getYaw(ekf.getRotation()) << std::endl;
}

/*************************************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "loggerEKF");
    ros::NodeHandle nh;

    LoggerEKF LoggerEKF(nh);

    ros::spin();

    return 0;
}

