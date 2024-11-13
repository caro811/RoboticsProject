#include <vector>
#include <tf/tf.h>
#include <robmovil_msgs/LandmarkArray.h>
#include <sensor_msgs/PointCloud.h>
#include "landmark_detector.h"

#define LANDMARK_DIAMETER 0.1 // metros

/*************************************************************************************************/
robmovil_ekf::LandmarkDetector::LandmarkDetector(ros::NodeHandle& _n) : n(_n), transform_received(false){
laser_sub = n.subscribe("/robot/front_laser/scan", 1, &LandmarkDetector::on_laser_scan, this);
    landmark_pub = n.advertise<robmovil_msgs::LandmarkArray>("/landmarks", 1);
    pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("/landmarks_pointcloud", 1);

    listener = boost::make_shared<tf::TransformListener>();

    n.param<bool>("print_laser_info", print_laser_info, false);

    n.param("robot_frame", robot_frame, std::string("base_link"));
    n.param("publish_robot_frame", publish_robot_frame, std::string("base_link"));
    n.param("laser_frame", laser_frame, std::string("front_laser"));

    if(print_laser_info)
        ROS_INFO_STREAM("publishing to frame " << publish_robot_frame);
}

/*************************************************************************************************/
float dist_euc(tf::Vector3 a, tf::Vector3 b){
    float dx = a.getX() - b.getX();
    float dy = a.getY() - b.getY();
    return sqrt(dx*dx + dy*dy);
}

/*************************************************************************************************/
void robmovil_ekf::LandmarkDetector::on_laser_scan(const sensor_msgs::LaserScanConstPtr& msg){
    if(!update_laser_tf(msg->header.stamp)){
        ROS_WARN_STREAM(laser_frame << " -> " << robot_frame << " transform not yet received, not publishing landmarks");
        return;
    }

    /* Convertir range,bearing a puntos cartesianos x,y,0.
     * Descartando aquellas mediciones por fuera de los rangos validos */
    std::vector<tf::Vector3> cartesian;

    /* Utilizar la informacion del mensaje para filtrar y convertir */
    float range_min = msg->range_min;
    float range_max = msg->range_max;

    float angle_min = msg->angle_min;
    float angle_increment = msg->angle_increment;

    for(int i=0; i < msg->ranges.size(); i++){
        /* p debe definirse con informacion valida y 
         * en coordenadas cartesianas */
        float range = msg->ranges[i];
        float angle = angle_min + i*angle_increment;

        if(range_min<range && range<range_max){
            tf::Vector3 p;
            p.setX(range*cos(angle));
            p.setY(range*sin(angle));
            p.setZ(0);

            /* convierto el punto en relacion al marco de referencia del laser al marco del robot */
            tf::Vector3 pr = laser_transform * p;

            if(print_laser_info)
                ROS_INFO_STREAM("p: " << p.x() << " " << p.y() << " " << p.z() << " pr: " << pr.x() << " " << pr.y() << " " << pr.z());

            cartesian.push_back(pr);
        }
    }

    /* Mensaje del arreglo de landmarks detectados */
    robmovil_msgs::LandmarkArray landmark_array;
    landmark_array.header.stamp = msg->header.stamp;
    landmark_array.header.frame_id = publish_robot_frame;

    /* VECTORES AUXILIARES: Pueden utilizar landmark_points para ir acumulando
     * mediciones cercanas */
    std::vector<tf::Vector3> landmark_points;

    tf::Vector3 robot_coord(0,0,0);

    // centroides estimados de los postes en coordenadas cartesianas
    std::vector<tf::Vector3> centroids;

    for(int i=0; i<cartesian.size(); i++){
        /* Acumular, de manera secuencial, mediciones cercanas (distancia euclidea) */
        /* Al terminarse las mediciones provenientes al landmark que se venia detectando,
         * se calcula la pose del landmark como el centroide de las mediciones */

        landmark_points.push_back(cartesian[i]);

        float min_dist = dist_euc(cartesian[i],robot_coord);
        int min_dist_i = 0;

        int j;
        for(j=i+1 ; j<cartesian.size() && dist_euc(cartesian[j-1],cartesian[j])<=LANDMARK_DIAMETER ; j++){
            landmark_points.push_back(cartesian[j]);

            if(min_dist > dist_euc(cartesian[j],robot_coord)){
                min_dist = dist_euc(cartesian[j],robot_coord);
                min_dist_i = j-i;
            }
        }
        i = j-1;

        if(print_laser_info)
            ROS_INFO_STREAM("landmark con " << landmark_points.size() << " puntos");

        /* Calcular el centroide de los puntos acumulados */
        tf::Vector3 centroid = landmark_points[min_dist_i] + landmark_points[min_dist_i]/min_dist*(LANDMARK_DIAMETER/2.0);

        if(print_laser_info)
            ROS_INFO_STREAM("landmark detectado (cartesianas): " << centroid.getX() << " " << centroid.getY() << " " << centroid.getZ());

        centroids.push_back(centroid);

        /* Convertir el centroide a coordenadas polares, construyendo el mensaje requerido */
        robmovil_msgs::Landmark landmark;

        float r = sqrt(centroid.getX()*centroid.getX() + centroid.getY()*centroid.getY()); // distancia desde el robot al centroide
        landmark.range = r;

        float a = atan2(centroid.getY(),centroid.getX()); // angulo de la recta que conecta al robot con el centroide
        landmark.bearing = a;

        /* se agrega el landmark en coordenadas polares */
        landmark_array.landmarks.push_back(landmark);

        if(print_laser_info)
            ROS_INFO_STREAM("landmark detectado (polares): " << i << ": " << landmark.range << " " << landmark.bearing);

        /* empiezo a procesar un nuevo landmark */
        landmark_points.clear();
    }

    /* Publicamos el mensaje de los landmarks encontrados */
    if(!landmark_array.landmarks.empty()){
        landmark_pub.publish(landmark_array);
        publish_pointcloud(landmark_array.header, centroids);
    }
}

/*************************************************************************************************/
bool robmovil_ekf::LandmarkDetector::update_laser_tf(const ros::Time& required_time){
  if (!listener->waitForTransform(robot_frame, laser_frame, required_time, ros::Duration(1)))
    return false;
  else
  {
    listener->lookupTransform(robot_frame, laser_frame, ros::Time(0), laser_transform);
    return true;
  }
}

/*************************************************************************************************/
void robmovil_ekf::LandmarkDetector::publish_pointcloud(const std_msgs::Header& header, const std::vector<tf::Vector3>& landmark_positions){
  sensor_msgs::PointCloud pointcloud;
  pointcloud.header.stamp = header.stamp;
  pointcloud.header.frame_id = header.frame_id;

  for (int i = 0; i < landmark_positions.size(); i++)
  {
    geometry_msgs::Point32 point;
    point.x = landmark_positions[i].getX();
    point.y = landmark_positions[i].getY();
    point.z = landmark_positions[i].getZ();
    pointcloud.points.push_back(point);
  }
  pointcloud_pub.publish(pointcloud);
}

