#include "robot_odometry.h"
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

using namespace robmovil;

// Constantes del robot
#define WHEEL_BASELINE_X 0.350
#define WHEEL_BASELINE_Y 0.350
#define WHEEL_RADIUS 0.050
#define ENCODER_TICKS 500.0

/*************************************************************************************************/
RobotOdometry::RobotOdometry(ros::NodeHandle& nh) : nh_(nh), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
    // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
    twist_sub_ = nh.subscribe("/robot/cmd_vel", 1, &RobotOdometry::on_velocity_cmd, this);

    vel_pub_front_left_  = nh.advertise<std_msgs::Float64>("/robot/front_left_wheel/cmd_vel", 1);
    vel_pub_front_right_ = nh.advertise<std_msgs::Float64>("/robot/front_right_wheel/cmd_vel", 1);
    vel_pub_rear_left_   = nh.advertise<std_msgs::Float64>("/robot/rear_left_wheel/cmd_vel", 1);
    vel_pub_rear_right_  = nh.advertise<std_msgs::Float64>("/robot/rear_right_wheel/cmd_vel", 1);

    encoder_sub_ = nh.subscribe("/robot/encoders", 1, &RobotOdometry::on_encoder_ticks, this);

    pub_odometry_ = nh.advertise<nav_msgs::Odometry>("/robot/odometry", 1);

    tf_broadcaster = boost::make_shared<tf::TransformBroadcaster>();

    ros::NodeHandle nhp("~");
    nhp.param<bool>("print_wheels_expected_speeds", print_wheels_expected_speeds, false);
    nhp.param<bool>("print_wheels_real_speeds", print_wheels_real_speeds, false);
    nhp.param<bool>("print_robot_real_speeds", print_robot_real_speeds, false);
}

/*************************************************************************************************/
// Cinematica inversa
void RobotOdometry::on_velocity_cmd(const geometry_msgs::Twist& twist)
{
    double vx = twist.linear.x;
    double vy = twist.linear.y;
    double wz = twist.angular.z;

    double lx = WHEEL_BASELINE_X / 2.0;
    double ly = WHEEL_BASELINE_Y / 2.0;

    double vFrontLeft  = (vx - vy - (lx+ly)*wz) / WHEEL_RADIUS; // w1
    double vFrontRight = (vx + vy + (lx+ly)*wz) / WHEEL_RADIUS; // w2
    double vRearLeft   = (vx + vy - (lx+ly)*wz) / WHEEL_RADIUS; // w3
    double vRearRight  = (vx - vy + (lx+ly)*wz) / WHEEL_RADIUS; // w4

    if(print_wheels_expected_speeds)
        ROS_INFO_STREAM("Expected w1=" << vFrontLeft << ", w2=" << vFrontRight << ", w3=" << vRearLeft << ", w4=" << vRearRight);

    // publish front left velocity
    {
        std_msgs::Float64 msg;
        msg.data = vFrontLeft;
        vel_pub_front_left_.publish(msg);
    }

    // publish front right velocity
    {
        std_msgs::Float64 msg;
        msg.data = vFrontRight;
        vel_pub_front_right_.publish(msg);
    }

    // publish rear left velocity
    {
        std_msgs::Float64 msg;
        msg.data = vRearLeft;
        vel_pub_rear_left_.publish(msg);
    }

    // publish rear right velocity
    {
        std_msgs::Float64 msg;
        msg.data = vRearRight;
        vel_pub_rear_right_.publish(msg);
    }
}

/*************************************************************************************************/
// Cinematica directa y odometría
void RobotOdometry::on_encoder_ticks(const robmovil_msgs::MultiEncoderTicks& encoder)
{
    // La primera vez que llega un mensaje de encoders
    // inicializo las variables de estado.
    if (not ticks_initialized_) {
        ticks_initialized_ = true;
        last_ticks_front_left_  = encoder.ticks[0].data;
        last_ticks_front_right_ = encoder.ticks[1].data;
        last_ticks_rear_left_   = encoder.ticks[2].data;
        last_ticks_rear_right_  = encoder.ticks[3].data;
        last_ticks_time = encoder.header.stamp;
        return;
    }

    int32_t delta_ticks_front_left  = encoder.ticks[0].data - last_ticks_front_left_;
    int32_t delta_ticks_front_right = encoder.ticks[1].data - last_ticks_front_right_;
    int32_t delta_ticks_rear_left   = encoder.ticks[2].data - last_ticks_rear_left_;
    int32_t delta_ticks_rear_right  = encoder.ticks[3].data - last_ticks_rear_right_;

    /* Utilizar este delta de tiempo entre momentos */
    double delta_t = (encoder.header.stamp - last_ticks_time).toSec();

    double lx = WHEEL_BASELINE_X/2.0;
    double ly = WHEEL_BASELINE_Y/2.0;

    double dw1 = 2.0*M_PI * delta_ticks_front_left  / ENCODER_TICKS;	// radianes
    double dw2 = 2.0*M_PI * delta_ticks_front_right / ENCODER_TICKS;	// radianes
    double dw3 = 2.0*M_PI * delta_ticks_rear_left   / ENCODER_TICKS;	// radianes
    double dw4 = 2.0*M_PI * delta_ticks_rear_right  / ENCODER_TICKS;	// radianes

    // Velocidades reales de las ruedas
    double omega_w1 = dw1 / delta_t;	// velocidad rueda 1 (rad/s)
    double omega_w2 = dw2 / delta_t;	// velocidad rueda 2 (rad/s)
    double omega_w3 = dw3 / delta_t;	// velocidad rueda 3 (rad/s)
    double omega_w4 = dw4 / delta_t;	// velocidad rueda 4 (rad/s)

    if(print_wheels_real_speeds)
        ROS_INFO_STREAM("Real w1=" << omega_w1 << ", w2=" << omega_w2 << ", w3=" << omega_w3 << ", w4=" << omega_w4);

    // Velocidad real del robot
    double vx = (  omega_w1 + omega_w2 + omega_w3 + omega_w4) * WHEEL_RADIUS / 4.0;
    double vy = (- omega_w1 + omega_w2 + omega_w3 - omega_w4) * WHEEL_RADIUS / 4.0;
    double wz = (- omega_w1 + omega_w2 - omega_w3 + omega_w4) * WHEEL_RADIUS / (4.0*(lx+ly));

    if(print_robot_real_speeds)
        ROS_INFO_STREAM("Real vx=" << vx << ", vy=" << vy << ", w=" << wz);

    // Desplazamientos en el marco del robot
    double delta_x_robot = vx * delta_t;
    double delta_y_robot = vy * delta_t;
    double delta_theta   = wz * delta_t;

    // y en el marco odom
    double delta_x_odom = delta_x_robot*cos(theta_) - delta_y_robot*sin(theta_);
    double delta_y_odom = delta_x_robot*sin(theta_) + delta_y_robot*cos(theta_);

    // Nueva pose
    /** Utilizar variables globales x_, y_, theta_ definidas en el .h */
    x_ += delta_x_odom;
    y_ += delta_y_odom;
    theta_ += delta_theta;

    // Construir el mensaje odometry
    nav_msgs::Odometry msg;

    msg.header.stamp = encoder.header.stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0;

    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.linear.z = 0;

    msg.twist.twist.angular.x = 0;
    msg.twist.twist.angular.y = 0;
    msg.twist.twist.angular.z = wz;

    pub_odometry_.publish(msg);

    // Actualizar las variables de estado
    last_ticks_front_left_  = encoder.ticks[0].data;
    last_ticks_front_right_ = encoder.ticks[1].data;
    last_ticks_rear_left_   = encoder.ticks[2].data;
    last_ticks_rear_right_  = encoder.ticks[3].data;
    last_ticks_time = encoder.header.stamp;

    // Mandar tambien un transform usando TF
    tf::Transform t;
    tf::poseMsgToTF(msg.pose.pose, t);
    tf_broadcaster->sendTransform(tf::StampedTransform(t, encoder.header.stamp, "odom", "base_link"));
}

