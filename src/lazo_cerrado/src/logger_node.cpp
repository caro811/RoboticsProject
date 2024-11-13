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
#include <boost/date_time/posix_time/posix_time.hpp>

/*************************************************************************************************/
class Logger{
    public:

        Logger(ros::NodeHandle& nh);

    private:

        ros::Subscriber robot_pose_sub_, ground_truth_sub_, goal_poses_sub_;

        std::ofstream robot_logfile_, goal_poses_logfile_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener transform_listener_;

        ros::Publisher odom_path_publisher, gt_path_publisher;
        nav_msgs::Path odom_path_msg, gt_path_msg;

        void handleRobotPose(const nav_msgs::Odometry& msg);
        void handleGoalPose(const geometry_msgs::PoseStamped& msg);
};

/*************************************************************************************************/
Logger::Logger(ros::NodeHandle& nh)
    : robot_logfile_("LC_poses.log"), transform_listener_( tfBuffer_ )
{
    // Paths descriptos en poses para visualizacion en RViz
    odom_path_publisher = nh.advertise<nav_msgs::Path>("/base_link_path", 1, true);
    gt_path_publisher = nh.advertise<nav_msgs::Path>("/base_link_gt_path", 1, true);

    robot_pose_sub_ = nh.subscribe("/robot/odometry", 1, &Logger::handleRobotPose, this);
    goal_poses_sub_ = nh.subscribe("/goal_pose", 1, &Logger::handleGoalPose, this);
}

/*************************************************************************************************/
void Logger::handleRobotPose(const nav_msgs::Odometry& msg){
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
    // Publicar en el log
    robot_logfile_ << msg.header.stamp.toSec() << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw( msg.pose.pose.orientation ) << std::endl;
}

/*************************************************************************************************/
void Logger::handleGoalPose(const geometry_msgs::PoseStamped& msg){
    goal_poses_logfile_ << msg.header.stamp.toSec() << " " << msg.pose.position.x << " " << msg.pose.position.y << " " << tf2::getYaw( msg.pose.orientation ) << std::endl;
}

/*************************************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "logger");
    ros::NodeHandle nh;

    Logger logger(nh);

    ros::spin();

    return 0;
}

