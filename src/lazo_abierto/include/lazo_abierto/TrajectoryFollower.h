#include <ros/ros.h>
#include <robmovil_msgs/Trajectory.h>

class TrajectoryFollower
{
  public:

    TrajectoryFollower(ros::NodeHandle& nh);

    void stop_timer() { timer_.stop(); }

  protected:

    /**
     * Callback que implementaran las clases derivadas.
     * 
     * @return
     *   false cuando termina de ejecutar la trayectoria, true en caso contrario.
     */
    virtual bool control(const ros::Time& t, double& vx, double& vy, double& w) = 0;

    const ros::Time& getInitialTime() const
    { return t0_; }

    const robmovil_msgs::Trajectory& getTrajectory() const
    { return current_trajectory_; }

    bool nextPointIndex(const ros::Time& time, size_t&) const;

  private:

    ros::NodeHandle& nh_;

    ros::Timer timer_;

    ros::Publisher cmd_pub_;

    ros::Subscriber trajectory_sub_;

    // tiempo en el cuál se comenzó a ejecutar el seguimiento actual.
    ros::Time t0_;

    // trayectoria actual.
    robmovil_msgs::Trajectory current_trajectory_;

  // funciones auxiliares

    void handleNewTrajectory(const robmovil_msgs::Trajectory& trajectory_msg);

    void timerCallback(const ros::TimerEvent& event);
};
