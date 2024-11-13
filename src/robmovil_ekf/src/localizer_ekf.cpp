#include <cmath>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <lazo_cerrado/tf_utils.hpp>
#include "localizer_ekf.h"

#define NEAREST_NEIGHBOR_RADIUS 1.0

/*************************************************************************************************/
robmovil_ekf::LocalizerEKF::LocalizerEKF(void) : EKFilter(3, 3, 3, 2, 2), transform_listener_( tfBuffer_ )
{
    ros::NodeHandle nhp("~");
    nhp.param<bool>("print_ekf_info", print_ekf_info, false);

    delta_t = 0;

    /* estado inicial */
    Vector x0(3);

    tf2::Transform pose;
    while(not lookupTransformSafe(tfBuffer_, "map", "odom", ros::Time::now(), pose)) ;

    x0(1) = pose.getOrigin().getX();
    x0(2) = pose.getOrigin().getY();
    x0(3) = tf2::getYaw(pose.getRotation());

    /* covarianza inicial */
    Matrix P(3, 3);

    P(1,1) = 0.1;
    P(1,2) = 0;
    P(1,3) = 0;

    P(2,1) = 0;
    P(2,2) = 0.1;
    P(2,3) = 0;

    P(3,1) = 0;
    P(3,2) = 0;
    P(3,3) = 0.1;

    if(print_ekf_info){
        ROS_DEBUG_STREAM("Initial state: " << x0);
        ROS_DEBUG_STREAM("Initial cov: " << P);
    }

    /* inicializa el filtro con estado y covarianza iniciales */
    init(x0, P); /* NOTA: esta llamada utiliza las referencias de x0 y P */
}

/*************************************************************************************************/
void robmovil_ekf::LocalizerEKF::set_map(const std::vector<tf::Point> observations){
    /* Nota: Se asume x = 0, el origen de coordenadas del mapa comienza donde el robot esta ahora,
     * los landmarks se guardar en relacion al mapa */
    for(unsigned int i = 0; i < observations.size(); i++){
        tf::Point landmark = observations[i];
        map_landmarks.push_back(landmark);

        if(print_ekf_info){
            ROS_DEBUG_STREAM("Landmark: " << landmark.getX() << "," << landmark.getY() << "," << landmark.getZ());
        }
    }
}

/*************************************************************************************************/
void robmovil_ekf::LocalizerEKF::set_delta_t(double delta){
    delta_t = delta;
}

/*************************************************************************************************/
/** Notificacion de una nueva medicion para su actualizacion.
 * 
 *    Se define la variable global correspondence_landmark con las coordenadas cartesianas
 *    del landmark perteneciente al mapa con el que la nueva medicion debera compararse */
bool robmovil_ekf::LocalizerEKF::set_measure(const Vector& new_measure_z){
    if(print_ekf_info)
        ROS_DEBUG_STREAM("New measure (polar): " << new_measure_z);

    /* Convertir medicion actual a landmark, en representation cartesiana y en referencia al mundo
     * (lugar en el momento en que comenzo el sistema) */
    tf::Point landmark_cartesian = measure2landmark(new_measure_z);

    if(print_ekf_info)
        ROS_DEBUG_STREAM("Landmark (cartesian): " << landmark_cartesian.getX() << ", " << landmark_cartesian.getY() << ", " << landmark_cartesian.getZ());

    /* Buscar en el mapa el landmark mas cercano a la observacion (establecimiento de correspondencia por nearest-neighbor) */
    bool found_correspondence = find_corresponding_landmark(landmark_cartesian, correspondence_landmark, NEAREST_NEIGHBOR_RADIUS);

    if(print_ekf_info){
        if(!found_correspondence){
            ROS_DEBUG_STREAM("No correspondence");
        }else{
            ROS_DEBUG_STREAM("Correspondence found: " << correspondence_landmark.getX() << ", " << correspondence_landmark.getY() << ", " << correspondence_landmark.getZ());
        }
    }

    return found_correspondence;
}

/*************************************************************************************************/
/** Jacobiano de A con respecto del estado (f) */
void robmovil_ekf::LocalizerEKF::makeBaseA(void){
    A(1,1) = 1;
    A(1,2) = 0;
    A(1,3) = 0;

    A(2,1) = 0;
    A(2,2) = 1;
    A(2,3) = 0;

    A(3,1) = 0;
    A(3,2) = 0;
    A(3,3) = 1;
}

/*************************************************************************************************/
/** Jacobiano de f con respecto del estado (x)
 *  La matriz se actualiza en cada ciclo de actualizacion de tiempo (prediccion) */
void robmovil_ekf::LocalizerEKF::makeA(void){
    // Sin cambios, es una matriz constante.

    double v = sqrt(u(1)*u(1)+u(2)*u(2));
    double alpha = atan2(u(2),u(1));

    A(1,3) = -sin(x(3)+alpha) * v * delta_t;
    A(2,3) =  cos(x(3)+alpha) * v * delta_t;

    if(print_ekf_info)
        ROS_DEBUG_STREAM("A: " << std::endl << A);
}

/*************************************************************************************************/
/** Jacobiano de f respecto de w */
void robmovil_ekf::LocalizerEKF::makeBaseW(void){
    W(1,1) = 1;
    W(1,2) = 0;
    W(1,3) = 0;

    W(2,1) = 0;
    W(2,2) = 1;
    W(2,3) = 0;

    W(3,1) = 0;
    W(3,2) = 0;
    W(3,3) = 1;
}

/*************************************************************************************************/
/** covarianza de w (ruido en f) */
void robmovil_ekf::LocalizerEKF::makeBaseQ(){
    Q(1,1) = 0.04;
    Q(1,2) = 0;
    Q(1,3) = 0;

    Q(2,1) = 0;
    Q(2,2) = 0.04;
    Q(2,3) = 0;

    Q(3,1) = 0;
    Q(3,2) = 0;
    Q(3,3) = 0.04;
}

/*************************************************************************************************/
/** Jacobiano de h respecto de x (valores iniciales) **/
void robmovil_ekf::LocalizerEKF::makeBaseH(void){
    H(1,1) = 1;
    H(1,2) = 0;
    H(1,3) = 0;

    H(2,1) = 0;
    H(2,2) = 1;
    H(2,3) = -1;
}

/*************************************************************************************************/
/** Jacobiano de h respecto de x
 ** La matriz se actualiza en cada ciclo de actualizacion de mediciones */
void robmovil_ekf::LocalizerEKF::makeH(void){
    /* Posicion del robot con respecto al MUNDO, es decir, con respecto
     * a donde se encontraba cuando comenzo el sistema */
    tf::Point robot_position(x(1), x(2), 0);
    //double robot_orientation = x(3);

    /* Obtener las coordenadas de correspondence_landmark con respecto al robot */
    tf::Point diff_robot_landmark = correspondence_landmark - robot_position;

    // Coordenadas cartesianas del landmark con respecto al robot
    if(print_ekf_info)
        ROS_DEBUG_STREAM("Relative_landmark: " << diff_robot_landmark.getX() << " " << diff_robot_landmark.getY() << " " << diff_robot_landmark.getZ());

    if(diff_robot_landmark.length2() < 0.001){
        H(1,1) = 1;
        H(1,2) = 0;
        H(1,3) = 0;

        H(2,1) = 0;
        H(2,2) = 1;
        H(2,3) = -1;

        ROS_ERROR_STREAM("Landmark too close to robot! Fake H used");
    }else{
        /* Calcular H en base al landmark del mapa relativo al robot */
        float dx = diff_robot_landmark.getX(); // px - x
        float dy = diff_robot_landmark.getY(); // py - y
        float distRL = diff_robot_landmark.length();//sqrt(dx*dx + dy*dy); // dist((px,py),(x,y))
        float distRL_2 = diff_robot_landmark.length2();//dx*dx + dy*dy; // dist((px,py),(x,y))^2 = ((px - x)^2 + (py - y)^2)

        H(1,1) = -dx/distRL; // -(px - x) / dist((px,py),(x,y))
        H(1,2) = -dy/distRL; // -(py - y) / dist((px,py),(x,y))
        H(1,3) = 0;

        H(2,1) =  dy/distRL_2; //  (py - y) / ((px - x)^2 + (py - y)^2)
        H(2,2) = -dx/distRL_2; // -(px - x) / ((px - x)^2 + (py - y)^2)
        H(2,3) = -1;
    }

    if(print_ekf_info)
        ROS_DEBUG_STREAM("H: " << std::endl << H);
}

/*************************************************************************************************/
/** Jacobiano de h respecto de v **/
void robmovil_ekf::LocalizerEKF::makeBaseV(void){
    V(1,1) = 1;
    V(1,2) = 0;

    V(2,1) = 0;
    V(2,2) = 1;

    if(print_ekf_info)
        ROS_DEBUG_STREAM("V: " << std::endl << V);
}

/*************************************************************************************************/
/** Covarianza de v **/
void robmovil_ekf::LocalizerEKF::makeBaseR(){
    R(1,1) = 0.001; // Var(rho) = (0.1)^2 (desvio de 10cm)
    R(1,2) = 0;

    R(2,1) = 0;
    R(2,2) = 0.00007615435; // Var(phi) = (0.5 * pi / 180)^2 (desvio de 0.5 grados)

    if(print_ekf_info)
        ROS_DEBUG_STREAM("R: " << std::endl << R);
}

/*************************************************************************************************/
/** Modelo de movimiento o proceso: x_t = f(x_{t-1}, u_{t-1}).
 *    Se debe utilizar el estado anterior y la entrada del modelo de movimiento
 *    para definir (predecir) la variable x */
void robmovil_ekf::LocalizerEKF::makeProcess(void){
    /* Utilizar las variables globales x_{t-1}, u y delta_t
     * para predecir el estado siguiente (prior state estimate).
     * Guardar el resultado en la variable global x */

    LocalizerEKF::Vector x_old(x); // x_{t-1}

    double v = sqrt(u(1)*u(1)+u(2)*u(2));
    double alpha = atan2(u(2),u(1));

    x(1) = x_old(1) + v * delta_t * cos(x_old(3)+alpha);
    x(2) = x_old(2) + v * delta_t * sin(x_old(3)+alpha);
    x(3) = angles::normalize_angle(x_old(3) + u(3)*delta_t);

    if(print_ekf_info)
        ROS_DEBUG_STREAM("Process model:" << std::endl << "X_{t-1}: " << x_old << std::endl << "X_t: " << x << std::endl << "delta_t: " << delta_t);
}

/*************************************************************************************************/
/** Modelo de sensado: z_t = h(x_t).
 *    
 *    Se debe utilizar la variable global correspondence_landmark previamente definida
 *    para definir la variable z con lo que deberia haber medido el sensor */
void robmovil_ekf::LocalizerEKF::makeMeasure(void){
    z = landmark2measure(correspondence_landmark);

    if(print_ekf_info)
        ROS_DEBUG_STREAM("Expected measure: " << z);
}

/*************************************************************************************************/
float point_euc_dist(tf::Point a, tf::Point b){
    float dx = a.getX() - b.getX();
    float dy = a.getY() - b.getY();
    return sqrt(dx*dx + dy*dy);
}

/*************************************************************************************************/
/** Recibe una medicion de landmark reciente con respecto al origen del mapa (lugar en que comenzo el sistema)
 ** y busca el landmark del mapa más cercano teniendo un radio delta como umbral.
 ** NOTA: El landmark perteneciente al mapa al cual hace referencia el landmark medido debe ser devuelto
 ******** por la referencia corresponding_landmark.
 **/
bool robmovil_ekf::LocalizerEKF::find_corresponding_landmark(const tf::Point& measured_landmark, tf::Point& corresponding_landmark, float delta_radio){
    /* Encontrar el landmark del mapa dentro del radio (delta_radio), mas cercano
     * a measured_landmark.
     * El resultado debe devolverse por la referencia corresponding_landmark */

    float min_distance = delta_radio;
    bool found = false;

    for(unsigned int i=0; i<map_landmarks.size(); i++){
        if(point_euc_dist(measured_landmark,map_landmarks[i]) <= min_distance){
            min_distance = point_euc_dist(measured_landmark,map_landmarks[i]);
            corresponding_landmark = map_landmarks[i];
            found = true;
        }
    }

    return found;
}

/*************************************************************************************************/
/** Convierte una medicion en coordenadas polares (relativa al sistema de coordenadas del robot) a un
 ** landmark absoluto en el sistema de coordenadas del mundo. **/
tf::Point robmovil_ekf::LocalizerEKF::measure2landmark(const LocalizerEKF::Vector& measure){
    /* Deben tomar la medicion realizada en referencia al robot y devolver
     * las coordenadas cartesianas en referencia al MUNDO.
     * Al utilizar el ultimo estado ESTIMADO para la conversion, el landmark calculado
     * corresponde a la prediccion de la posicion del landmark */

    /* Measurement parameters */
    double rho = measure(1);
    double phi = measure(2);

    /* Ultimo estado estimado del robot */
    double robot_x = x(1);
    double robot_y = x(2);
    double robot_theta = x(3);

    /* Considerar la orientacion del robot
     * y el angulo de la medicion con respecto al robot */
    float absolute_angle = robot_theta + phi;

    /* TOMAR EN CUENTA LA POSICION x,y DEL ROBOT */
    tf::Point predicted_landmark;
    predicted_landmark.setX(robot_x + cos(absolute_angle)*rho);
    predicted_landmark.setY(robot_y + sin(absolute_angle)*rho);
    predicted_landmark.setZ(0);

    return predicted_landmark;
}

/*************************************************************************************************/
/** Convierte un landmark (en coordenadas cartesianas en referencia al mapa) a una medicion (coordenadas polares)
 ** en referencia al sistema de coordenadas del robot */
robmovil_ekf::LocalizerEKF::Vector robmovil_ekf::LocalizerEKF::landmark2measure(const tf::Point& landmark){
    /* Deben tomar un landmark del mapa (cartesianas) y devolver
     * las coordenadas polares con las que deberia haber medido dicho landmark
     * utilizando como posicion actual del robot: la ultima ESTIMACION del estado (x)
     * NOTA: Para esto deberan primero "traducir" la posicion del landmark en respecto
     *             del robot. CONSIDERAR la inversa de la transformacion que va desde el marco
     *             del mundo al marco del robot ( transform_world_robot.inverse() ).
     */

    if(print_ekf_info)
        ROS_DEBUG_STREAM("Robot pose: " << x(1) << " " << x(2) << " " << x(3));

    /* Pose del robot contruida utilizando el estado estimado hasta este momento
     * (prior estate estimate) */
    tf::Transform transform_world_robot;
    transform_world_robot.setOrigin(tf::Vector3(x(1),x(2),0));
    transform_world_robot.setRotation(tf::createQuaternionFromYaw(x(3)));

    tf::Point relative_landmark = transform_world_robot.inverse() * landmark;

    if(print_ekf_info)
        ROS_DEBUG_STREAM("relative_landmark: " << relative_landmark.getX() << ", " << relative_landmark.getY() << ", " << relative_landmark.getZ());

    LocalizerEKF::Vector measure(2);

    measure(1) = point_euc_dist(relative_landmark,tf::Point(0,0,0)); // Calculo del rho

    relative_landmark.normalize();
    measure(2) = angles::normalize_angle(atan2(relative_landmark.getY(),relative_landmark.getX())); // Calculo del phi

    return measure;
}

