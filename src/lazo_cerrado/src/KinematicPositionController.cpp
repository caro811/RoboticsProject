#include <angles/angles.h>
#include <lazo_cerrado/tf_utils.hpp>
#include "KinematicPositionController.h"

/*************************************************************************************************/
KinematicPositionController::KinematicPositionController(ros::NodeHandle& nh) :
    TrajectoryFollower(nh), transform_listener_( tfBuffer_ )
{
    expected_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 1);

    ros::NodeHandle nhp("~");

    nhp.param<bool>("print_trajectory_follow_info", print_trajectory_follow_info, false);

	nhp.param<double>("K1", K1, 0.5);
    nhp.param<double>("K2", K2, 0.5);
    nhp.param<double>("K3", K3, 0.5);
    nhp.param<double>("LOOKAHEAD", LOOKAHEAD, 0.5);

    nhp.param<bool>("EKF", EKF, false);
}

/*************************************************************************************************/
bool KinematicPositionController::getCurrentPose(const ros::Time& t, double& x, double& y, double& a){
    tf2::Transform pose;
    if(EKF){
        if(not lookupTransformSafe(tfBuffer_, "map", "base_link_ekf", t, pose))
            return false;
    }else{
        if(not lookupTransformSafe(tfBuffer_, "map", "base_link", t, pose))
            return false;
    }

    x = pose.getOrigin().getX();
    y = pose.getOrigin().getY();

    a = tf2::getYaw(pose.getRotation());

    return true;
}

/*************************************************************************************************/
bool KinematicPositionController::control(const ros::Time& t, double& vx, double& vy, double& w){
    // Se obtiene la pose actual estimada
    double current_x, current_y, current_a;
    if( not getCurrentPose(t, current_x, current_y, current_a) )
        return true;

    // Se obtiene la pose objetivo actual a seguir
    double goal_x, goal_y, goal_a;
    if( not getCurrentGoal(t, goal_x, goal_y, goal_a) )
        return false;

    // Publicación de la pose objetivo para visualizar en RViz
    publishCurrentGoal(t, goal_x, goal_y, goal_a);

    // Calcular velocidad lineal y angular
    double Idx = goal_x - current_x;
    double Idy = goal_y - current_y;
    double Ida = angles::normalize_angle(goal_a - current_a);

    double rho = sqrt(Idx*Idx + Idy*Idy);
    double alpha = atan2(Idy,Idx) - current_a;
    double theta = Ida;

    vx = K1 * rho*cos(alpha);
    vy = K2 * rho*sin(alpha);
    w  = K3 * theta;

    // Debug
    if(print_trajectory_follow_info){
        ROS_INFO_STREAM("Current goal x: " << goal_x << " y: " << goal_y << " a: " << goal_a);
        ROS_INFO_STREAM(" vx: " << vx << " vy: " << vy << " w: " << w);
    }

    return true;
}

/*************************************************************************************************/
/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1){
    return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
}

/*************************************************************************************************/
bool KinematicPositionController::getPursuitBasedGoal(const ros::Time& t, double& x, double& y, double& a){
    // Los obtienen los valores de la posicion y orientacion actual.
    double current_x, current_y, current_a;
    if(not getCurrentPose(t, current_x, current_y, current_a))
        return true;

    if(print_trajectory_follow_info)
        ROS_INFO_STREAM("Current position x: " << current_x << " y: " << current_y << " a: " << current_a);

    // Se obtiene la trayectoria requerida.
    const robmovil_msgs::Trajectory& trajectory = getTrajectory();

    /* Se recomienda encontrar el waypoint de la trayectoria más cercano al robot en términos de x,y
     * y luego buscar el primer waypoint que se encuentre a una distancia predefinida de lookahead en x,y */
    double dmin = -1.0;
    double xmin = -1.0;
    double ymin = -1.0;
    int imin = -1;

    // Cuantos waypoints se puede haber movido?
    int buscar_hasta = previous_closest + trajectory.points.size()/10; // Lo fijamos en el anterior más un 10% del total de puntos
    if(buscar_hasta > trajectory.points.size()){
        buscar_hasta = trajectory.points.size();
    }

    for(unsigned int i = previous_closest; i < buscar_hasta; i++){ // No buscamos entre todos, solo entre los más cercanos al anterior.
        const robmovil_msgs::TrajectoryPoint& wpoint = trajectory.points[i];
        double wpoint_x = wpoint.transform.translation.x;
        double wpoint_y = wpoint.transform.translation.y;
        //double wpoint_a = tf2::getYaw(wpoint.transform.rotation);

        double d = dist2(current_x, current_y, wpoint_x, wpoint_y);

        if(dmin==-1 || d<dmin){
            dmin = d;
            xmin = wpoint_x;
            ymin = wpoint_y;
            imin = i;
        }
    }
/*    if(print_trajectory_follow_info)
        ROS_INFO_STREAM("Closest point to current position (i): " << imin);*/
    previous_closest = imin;

    bool found_goal = false;
    for(unsigned int j = imin+1; j < trajectory.points.size() && !found_goal; j++){
        const robmovil_msgs::TrajectoryPoint& wpoint = trajectory.points[j];
        double wpoint_x = wpoint.transform.translation.x;
        double wpoint_y = wpoint.transform.translation.y;
        double wpoint_a = tf2::getYaw(wpoint.transform.rotation);

        double d = dist2(xmin, ymin, wpoint_x, wpoint_y);

        if(d >= LOOKAHEAD){
            found_goal = true;
            x = wpoint_x;
            y = wpoint_y;
            a = wpoint_a;
        }
    }

    /* retorna true si es posible definir un goal, false si se termino la trayectoria y no quedan goals. */
    return found_goal;
}

