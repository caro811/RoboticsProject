#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include "localizer.h"

/*************************************************************************************************/
robmovil_ekf::Localizer::Localizer(ros::NodeHandle& n) :
    u(3)
{
    u(1) = u(2) = u(3) = 0;
    set_map = true;

    // Get node parameters
    n.param<std::string>("base_link_frame", base_frame_, "base_link_ekf");
    n.param<std::string>("ekf_frame", map_frame_, "map");
    n.param<std::string>("laser_frame", laser_frame_, "front_laser");

    n.param<bool>("print_localizer_info", print_localizer_info, false);

    n.param<bool>("only_prediction", only_prediction, false);

    posts_sub = n.subscribe("/posts", 1, &Localizer::get_posts, this);
    landmark_sub = n.subscribe("/landmarks", 1, &Localizer::on_landmark_array, this);

    odo_sub = n.subscribe("/robot/odometry", 1, &Localizer::on_odometry, this);
    pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1);
  
    while( (t = ros::Time::now()) == ros::Time(0) ); // se espera por el primer mensaje de 'clock'

    prediction_timer = n.createTimer(ros::Rate(10), &Localizer::prediction_event, this, false);
}

/*************************************************************************************************/
/**
 * Prediccion hecha periódicamente, en caso de que no haya habido sensado por cierto tiempo
 */

void robmovil_ekf::Localizer::prediction_event(const ros::TimerEvent& event)
{
  if(print_localizer_info)
    ROS_DEBUG_STREAM("Prediction Event");

  prediction(event.current_real);
  
  /* publicar estimado actual */
  publish_estimate(event.current_real);
}

/*************************************************************************************************/
void robmovil_ekf::Localizer::prediction(const ros::Time& now)
{
  if(print_localizer_info)
    ROS_DEBUG_STREAM("Prediction");

  /* avanzar el tiempo del filtro desde el tiempo anterior al actual */
  advance_time(now);

  if(print_localizer_info)
    ROS_DEBUG_STREAM("t is now: " << t);

  /* hacer prediccion */
  ekf.timeUpdateStep(u);

  LocalizerEKF::Vector x = ekf.getX();
  LocalizerEKF::Matrix P = ekf.calculateP();

  if(print_localizer_info){
    ROS_DEBUG_STREAM("Predicted estimate: " << x);
    ROS_DEBUG_STREAM("Predicted covariance: " << P);
  }
}

/*************************************************************************************************/
/* Toma las posiciones de los postes dadas por topico /posts y crea el mapa.
   Las posiciones vienen dadas en el marco de referencia del mapa.
*/
void robmovil_ekf::Localizer::get_posts(const geometry_msgs::PoseArrayConstPtr& posts)
{
    if(set_map){
        if(print_localizer_info){
            ROS_DEBUG_STREAM("posts: " << t);

            ROS_INFO("Creating map");
        }

        std::vector<tf::Point> map;
        for(unsigned int i = 0; i < posts->poses.size(); i++){
            // Posiciones respecto a map
            tf::Point p;
            p.setX(posts->poses[i].position.x);
            p.setY(posts->poses[i].position.y);
            p.setZ(0);

            // Add point
            map.push_back(p);
        }

        ekf.set_map(map);

        set_map = false;
    }
}

/*************************************************************************************************/
void robmovil_ekf::Localizer::on_landmark_array(const robmovil_msgs::LandmarkArrayConstPtr& msg)
{
    if(set_map)
        return;

    if(only_prediction)
        return;

    if(print_localizer_info)
        ROS_DEBUG_STREAM("on_landmark_array: " << t);

    /* Se apaga el timer que predice periodicamente dado que
     * se predice y actualiza dadas las mediciones recibidas */
    prediction_timer.stop();

    // Prediccion avanzando el tiempo al momento del sensado
    prediction(msg->header.stamp);

    /* hacer update(s) */
    for(unsigned int i = 0; i < msg->landmarks.size(); i++){
        LocalizerEKF::Vector z(2);
    
        z(1) = msg->landmarks[i].range;
        z(2) = msg->landmarks[i].bearing;
    
        if(print_localizer_info)
            ROS_DEBUG_STREAM("Measurement update with: " << z);
    
        bool found_correspondence = ekf.set_measure(z);

        if(found_correspondence){
            ekf.measureUpdateStep(z);

            if(print_localizer_info){
                ROS_DEBUG_STREAM("Measure updated estimate: " << ekf.getX());
                ROS_DEBUG_STREAM("Measure updated covariance: " << ekf.calculateP());
            }
        }
    }

    /* publicar estimado actual */
    publish_estimate(msg->header.stamp);

    /* prendo el timer de prediccion, por si no llega un sensado a tiempo y asi todavia estaria prediciendo hasta que vuelva a haber un sensado */
    prediction_timer.start();
}

/*************************************************************************************************/
void robmovil_ekf::Localizer::on_odometry(const nav_msgs::OdometryConstPtr& msg)
{
    u(1) = msg->twist.twist.linear.x;
    u(2) = msg->twist.twist.linear.y;
    u(3) = msg->twist.twist.angular.z;

    if(print_localizer_info){
        ROS_DEBUG_STREAM("Received linear velocity X: " << u(1));
        ROS_DEBUG_STREAM("Received linear velocity Y: " << u(2));
        ROS_DEBUG_STREAM("Received angular velocity:  " << u(3));
    }
}

/*************************************************************************************************/
void robmovil_ekf::Localizer::advance_time(const ros::Time& now)
{
  double delta_t = (now - t).toSec();
  t = now;

  if (delta_t < 0) ROS_ERROR_STREAM("Negative delta_t! " << delta_t);
  else ekf.set_delta_t(delta_t);

  if(print_localizer_info)
    ROS_DEBUG_STREAM("Time advanced by " << delta_t);
}

/*************************************************************************************************/
void robmovil_ekf::Localizer::publish_estimate(const ros::Time& now)
{
  LocalizerEKF::Vector x = ekf.getX();
  LocalizerEKF::Matrix P = ekf.calculateP();

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = now;
  pose_msg.header.frame_id = map_frame_;

  /* build covariance */
  pose_msg.pose.covariance.assign(0);

  /* XY/XY covariance */
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      pose_msg.pose.covariance[i * 6 + j] = P(i + 1, j + 1);

  /* XY/theta covariance */
  pose_msg.pose.covariance[0 * 6 + 5] = P(1,3);
  pose_msg.pose.covariance[1 * 6 + 5] = P(2,3);

  /* theta/XY covariance */
  pose_msg.pose.covariance[5 * 6 + 0] = P(3,1);
  pose_msg.pose.covariance[5 * 6 + 1] = P(3,2);

  /* theta/theta covariance */
  pose_msg.pose.covariance[5 * 6 + 5] = P(3,3);

  /* pongo en -1 los elementos de z, y los dos angulos sin usar, dado que estoy trabajando en 2D */
  pose_msg.pose.covariance[2 * 6 + 2] = -1;
  pose_msg.pose.covariance[3 * 6 + 3] = -1;
  pose_msg.pose.covariance[4 * 6 + 4] = -1;

  /* set pose */
  tf2::Transform map_to_base_link;
  map_to_base_link.setOrigin(tf2::Vector3(x(1), x(2), 0));
  map_to_base_link.setRotation(tf2::Quaternion(tf2::Vector3(0,0,1),x(3)));
  tf2::toMsg(map_to_base_link, pose_msg.pose.pose);

  /* publish */
  pose_pub.publish( pose_msg );

  // publish TF transform between ekf frame and odom frame
  geometry_msgs::TransformStamped map_to_base_link_msg;
  map_to_base_link_msg.header.frame_id = map_frame_;
  map_to_base_link_msg.header.stamp = now;
  map_to_base_link_msg.child_frame_id = base_frame_;

  map_to_base_link_msg.transform = tf2::toMsg( map_to_base_link );

  transform_broadcaster_.sendTransform( map_to_base_link_msg );
}

