#ifndef __ROBMOVIL_EKF_LOCALIZEREKF_H__
#define __ROBMOVIL_EKF_LOCALIZEREKF_H__

#include <ros/ros.h>
#include <kalman/ekfilter.hpp>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/**
 * TODO: abstraerse de los tipos internos (solo habria que lidiar con eso en los metodos a overridear
 */

namespace robmovil_ekf
{
  typedef Kalman::EKFilter<double,1,false,false,false> EKFilter;

  /**
   * Localizador EKF
   *
   * Estado:   X = (x, y, theta)
   * Input:    U = (vx, vy, omega)
   * Medicion: Z = (rho, phi)
   * Se incluyen dos ruidos aditivos para modelo de proceso y sensado, respectivamente
   * - ruido del actuador: w
   * - ruido del sensor:   v
   */
  class LocalizerEKF : public EKFilter
  {
    public:
      LocalizerEKF(void);

      /* Jacobiano de f() respecto a X */
      void makeBaseA(void);
      void makeA(void);

      /* Jacobiano de f() respecto a w */
      void makeBaseW(void);
//      void makeW(void);

      /* Covarianza de w */
      void makeBaseQ(void);

      /* Jacobiano de h() con respecto a X */
      void makeBaseH(void);
      void makeH(void);

      /* Jacobiano de h() con respecto a v */
      void makeBaseV(void);

      /* Covarianza de v */
      void makeBaseR(void);

      /* modelo de proceso: f */
      void makeProcess(void);

      /* modelo de sensado: h */
      void makeMeasure(void);

      void set_map(const std::vector<tf::Point> observations);
      void set_delta_t(double delta); // seconds
      bool set_measure(const Vector& measured_z);

    private:

      /* Encuentra el landmark del mapa mas cercano al provisto y lo devuelve por corresponding_landmark */
      bool find_corresponding_landmark(const tf::Point& measured_landmark, tf::Point& corresponding_landmark, float delta);

      /* Convierte una medicion en coordenadas polares (relativa al sistema de coordenadas del robot) a un
       * landmark absoluto en el sistema de coordenadas del mundo */
      tf::Point measure2landmark(const Vector& measure);

      /* Convierte un landmark (en coordenadas cartesianas en referencia al mapa) a una medicion (coordenadas polares)
       *  en referencia al sistema de coordenadas del robot */
      LocalizerEKF::Vector landmark2measure(const tf::Point& landmark);

      /* Mapa compuesto de landmarks en coordenadas cartesianas en referencia
       * al lugar donde comenzo a funcionar el sistema */
      std::vector<tf::Point> map_landmarks;

      /* Delta tiempo desde la ultima prediccion (time update) */
      double delta_t;
      
      /* Landmark del mapa (en coordenadas cartesianas en referencia al lugar donde empezo el sistema)
       * el cual corresponde con la ultima medicion ingresada (por set_measure) */
      tf::Point correspondence_landmark;

      tf2_ros::Buffer tfBuffer_;
      tf2_ros::TransformListener transform_listener_;

      bool print_ekf_info;
  };
}

#endif // __ROBMOVIL_EKF_LOCALIZEREKF_H__
