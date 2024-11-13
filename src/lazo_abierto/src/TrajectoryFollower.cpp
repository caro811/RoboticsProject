#include <lazo_abierto/TrajectoryFollower.h>
#include <geometry_msgs/Twist.h>

TrajectoryFollower::TrajectoryFollower(ros::NodeHandle& nh) : nh_( nh )
{
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);

    trajectory_sub_ = nh.subscribe("/robot/trajectory", 1, &TrajectoryFollower::handleNewTrajectory, this);
}

void TrajectoryFollower::handleNewTrajectory(const robmovil_msgs::Trajectory& trajectory_msg){
    ROS_INFO("New trajectory recived");

    while((t0_ = ros::Time::now()) == ros::Time(0))
        ;

    ROS_INFO_STREAM("t0: " << t0_);
    ROS_INFO_STREAM("trajectory size: " << trajectory_msg.points.size());

    current_trajectory_ = trajectory_msg;

    timer_ = nh_.createTimer(ros::Duration(0.01), &TrajectoryFollower::timerCallback, this);
}

void TrajectoryFollower::timerCallback(const ros::TimerEvent& event){
    const ros::Time& t = event.current_real;
    ROS_INFO_STREAM("t: " << t);

    // Aplicar la ley de control
    double vx, vy, w;
    if(not control(t, vx, vy, w)) {
        ROS_INFO("Trajectory finished");
        timer_.stop();

        geometry_msgs::Twist cmd;
        cmd_pub_.publish(cmd); // se detiene luego de terminar el trayecto
        return;
    }

    // Crear mensaje
    geometry_msgs::Twist cmd;

    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.linear.z = 0;

    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = w;

    cmd_pub_.publish(cmd);
}

bool TrajectoryFollower::nextPointIndex(const ros::Time& time, size_t& next_point_idx) const{
    size_t idx = 0;
    for(const robmovil_msgs::TrajectoryPoint& point : current_trajectory_.points){
        if(time < t0_ + point.time_from_start){
            next_point_idx = idx;
            return true;
        }
        idx++;
    }

    return false;
}

