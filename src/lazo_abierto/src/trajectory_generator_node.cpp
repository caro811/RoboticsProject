#include <ros/ros.h>
#include <robmovil_msgs/Trajectory.h>
#include <robmovil_msgs/TrajectoryPoint.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <XmlRpcValue.h>
#include <angles/angles.h>

#include <vector>

/*********************************************************************************************************************/
// Potencias (2 y 3)
#define ala2(x) ((x) * (x))
#define ala3(x) ((x) * (x) * (x))

// Dimensiones
#define  TIME 0
#define     X 1
#define     Y 2
#define THETA 3

// Variable global para debug
bool print_trajectory;

/*********************************************************************************************************************/
// Auxiliar para la diferencia de angulos
double dist_theta(double theta_b, double theta_a){
    theta_a = angles::normalize_angle(theta_a);
    theta_b = angles::normalize_angle(theta_b);

    double d = theta_b - theta_a;

    if(d < M_PI){
        return d;
    }else{
        return (d - 2.0*M_PI);
    }
}

/*********************************************************************************************************************/
// Obtener las derivadas
void get_derivadas(std::vector<std::vector<double>>& wpoints, int total_points, double derivada[][4]){
    derivada[0][X]     = 0.0;
    derivada[0][Y]     = 0.0;
    derivada[0][THETA] = 0.0;

    for(int n_point = 1; n_point < total_points - 1; n_point++){
        // anterior
        double t_a = wpoints[n_point - 1][TIME];
        double x_a = wpoints[n_point - 1][X];
        double y_a = wpoints[n_point - 1][Y];
        double a_a = wpoints[n_point - 1][THETA];

        // siguiente
        double t_c = wpoints[n_point + 1][TIME];
        double x_c = wpoints[n_point + 1][X];
        double y_c = wpoints[n_point + 1][Y];
        double a_c = wpoints[n_point + 1][THETA];

        // deltas
        double dt_ac = t_c - t_a;
        double dx_ac = x_c - x_a;
        double dy_ac = y_c - y_a;
        double da_ac = dist_theta(a_c, a_a);

        // derivadas
        derivada[n_point][X]     = dx_ac / dt_ac;
        derivada[n_point][Y]     = dy_ac / dt_ac;
        derivada[n_point][THETA] = da_ac / dt_ac;
    }

    derivada[total_points - 1][X]     = 0.0;
    derivada[total_points - 1][Y]     = 0.0;
    derivada[total_points - 1][THETA] = 0.0;
}

/*********************************************************************************************************************/
// Obtener una dimension del spline
void get_spline(int dim, double &x, double &vx, int n, std::vector<std::vector<double>>& wpoints, double vel[][4], double t_now){
    double x0 = wpoints[n][dim];
    double x1 = wpoints[n+1][dim];

    double t0 = wpoints[n][TIME];
    double t1 = wpoints[n+1][TIME];
    double dt = t1 - t0;

    double v0 = vel[n][dim];
    double v1 = vel[n+1][dim];

    double a0 = x0;
    double a1 = v0;
    double a2 = 3*(x1 - x0)/ala2(dt) - (2*v0 + v1)/dt;
    double a3 = -2*(x1 - x0)/ala3(dt) + (v1 + v0)/ala2(dt);

    double t = t_now - t0;
    x = a0 + a1*t + a2*ala2(t) + a3*ala3(t);
    vx = a1 + 2*a2*t + 3*a3*ala2(t);
}

/*********************************************************************************************************************/
// Publicar un waypoint (pose y velocidades) como mensaje
void set_waypoints_speed(robmovil_msgs::Trajectory& trajectory_msg, nav_msgs::Path& path_msg, double t, double x, double y, double a, double vx, double vy, double va){
    /* se crean los waypoints de la trajectoria */
    robmovil_msgs::TrajectoryPoint point_msg;

    point_msg.time_from_start = ros::Duration(t);

    point_msg.transform.translation.x = x;
    point_msg.transform.translation.y = y;
    point_msg.transform.translation.z = 0;

    point_msg.transform.rotation = tf::createQuaternionMsgFromYaw(a);

    point_msg.velocity.linear.x = vx;
    point_msg.velocity.linear.y = vy;
    point_msg.velocity.linear.z = 0;

    point_msg.velocity.angular.x = 0;
    point_msg.velocity.angular.y = 0;
    point_msg.velocity.angular.z = va;

    trajectory_msg.points.push_back(point_msg);

    /* Construccion de la trayectoria como un nav_msgs::Path para su visualizacion en RViz */
    geometry_msgs::PoseStamped stamped_pose_msg;

    stamped_pose_msg.header.stamp = path_msg.header.stamp;
    stamped_pose_msg.header.frame_id = path_msg.header.frame_id;

    stamped_pose_msg.pose.position.x = x;
    stamped_pose_msg.pose.position.y = y;
    stamped_pose_msg.pose.position.z = 0;

    stamped_pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(a);

    path_msg.poses.push_back(stamped_pose_msg);

    // Debug
    if(print_trajectory){
        ROS_INFO_STREAM(
            "\n t " << t
            << "\n" <<
            "\t x " << x << ",\t  y " << y << ",\t  a " << a
            << "\n" <<
            "\t vx " << vx << ",\t vy " << vy << ",\t va " << va
        );
    }
}

/*********************************************************************************************************************/
// Creacion de la trayectoria cuadrada pedida
void build_square_trajectory(int total_points, double stepping, double total_time, robmovil_msgs::Trajectory& trajectory_msg, nav_msgs::Path& path_msg){
    /*********************************************/
    // Construir los waypoints
    std::vector<std::vector<double>> wpoints;

    int total_waypoints_lado = (total_points-1)/4;

    // Primer lado
    for(int i = 0; i < total_waypoints_lado; i++){
        std::vector<double> p;

        double f = i / (double)total_waypoints_lado;

        p.push_back(total_time * i / (double)(total_points-1));
        p.push_back(-2.0 + 4.0*f);          // pos_inicial + long_lado*fraccion_actual
        p.push_back(2.0);                   // Fijo
        p.push_back(M_PI/2.0 - M_PI/2.0*f); // orientacion_inicial + delta_orientacion*fraccion_actual

        wpoints.push_back(p);
    }
    // Segundo lado
    for(int i = total_waypoints_lado; i < 2*total_waypoints_lado; i++){
        std::vector<double> p;

        double f = (i - total_waypoints_lado) / (double)total_waypoints_lado;

        p.push_back(total_time * i / (double)(total_points-1));
        p.push_back(2.0);                   // Fijo
        p.push_back(2.0 - 4.0*f);           // pos_inicial + long_lado*fraccion_actual
        p.push_back(0.0 - M_PI/2.0*f);      // orientacion_inicial + delta_orientacion*fraccion_actual

        wpoints.push_back(p);
    }
    // Tercer lado
    for(int i = 2*total_waypoints_lado; i < 3*total_waypoints_lado; i++){
        std::vector<double> p;

        double f = (i - 2*total_waypoints_lado) / (double)total_waypoints_lado;

        p.push_back(total_time * i / (double)(total_points-1));
        p.push_back(2.0 - 4.0*f);           // pos_inicial + long_lado*fraccion_actual
        p.push_back(-2.0);                  // Fijo
        p.push_back(-M_PI/2.0 - M_PI/2.0*f);// orientacion_inicial + delta_orientacion*fraccion_actual

        wpoints.push_back(p);
    }
    // Cuarto lado
    for(int i = 3*total_waypoints_lado; i < 4*total_waypoints_lado; i++){
        std::vector<double> p;

        double f = (i - 3*total_waypoints_lado) / (double)total_waypoints_lado;

        p.push_back(total_time * i / (double)(total_points-1));
        p.push_back(-2.0);                  // Fijo
        p.push_back(-2.0 + 4.0*f);          // pos_inicial + long_lado*fraccion_actual
        p.push_back(-M_PI - M_PI/2.0*f);    // orientacion_inicial + delta_orientacion*fraccion_actual

        wpoints.push_back(p);
    }
    // Punto final
    {
        std::vector<double> p;

        p.push_back(total_time);
        p.push_back(-2.0);
        p.push_back(2.0);
        p.push_back(-3*M_PI/2.0);

        wpoints.push_back(p);
    }
    /*********************************************/

    // Construir los polinomios y obtener las velocidades en cada punto
    double derivada[total_points][4];
    get_derivadas(wpoints,total_points,derivada);

    for(int n_point=0; n_point<total_points-1; n_point++){
        double x_now = wpoints[n_point][X];
        double y_now = wpoints[n_point][Y];
        double a_now = wpoints[n_point][THETA];

        double initial_time = wpoints[n_point][TIME];
        double final_time = wpoints[n_point+1][TIME];

        for(double t=initial_time; t<=final_time; t+=stepping){
            double x, y, a;
            double vx_M, vy_M, va_M;
            get_spline(X, x, vx_M, n_point, wpoints, derivada, t);
            get_spline(Y, y, vy_M, n_point, wpoints, derivada, t);
            get_spline(THETA, a, va_M, n_point, wpoints, derivada, t);

            // Transformar las velocidades al marco del robot
            double vx, vy, va;
            double alfa = atan2(vy_M,vx_M);
            double modv = sqrt(vx_M*vx_M + vy_M*vy_M);
            vx = modv*cos(dist_theta(alfa, a_now));
            vy = modv*sin(dist_theta(alfa, a_now));
            va = va_M;

            // Publicar mensaje
            set_waypoints_speed(trajectory_msg, path_msg, t, x, y, a, vx, vy, va);

            // Actualizar pose
            x_now = x;
            y_now = y;
            a_now = a;
        }
    }

    // Velocidades en el ultimo punto
    double x = wpoints[total_points-1][X];
    double y = wpoints[total_points-1][Y];
    double a = wpoints[total_points-1][THETA];
    double t = wpoints[total_points-1][TIME];
    double vx = 0.0;
    double vy = 0.0;
    double va = 0.0;

    set_waypoints_speed(trajectory_msg, path_msg, t, x, y, a, vx, vy, va);
}

/*********************************************************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    nhp.param<bool>("print_trajectory", print_trajectory, false);

    ros::Publisher trajectory_publisher = nh.advertise<robmovil_msgs::Trajectory>("/robot/trajectory", 1, true);

    // Path descripto en poses para visualizacion en RViz
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/ground_truth/target_path", 1, true);

    robmovil_msgs::Trajectory trajectory_msg;
    nav_msgs::Path path_msg;

    trajectory_msg.header.seq = 0;
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.frame_id = "map";

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    int total_points;
    double stepping;
    double total_time;

    nhp.param<int>("total_points", total_points, 9);
    if(total_points < 5 || total_points % 4 != 1){
        ROS_ERROR("total_points debe ser un multiplo de 4 +1, mayor o igual a 5.");
        return 1;
    }
    nhp.param<double>("stepping", stepping, 0.1);
    nhp.param<double>("total_time", total_time, 100);

    build_square_trajectory(total_points, stepping, total_time, trajectory_msg, path_msg);

    trajectory_publisher.publish(trajectory_msg);
    path_publisher.publish(path_msg);

    ros::spin();

    return 0;
}

