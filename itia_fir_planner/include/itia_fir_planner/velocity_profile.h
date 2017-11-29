#ifndef __fir_planner_velocity_profile__
#define __fir_planner_velocity_profile__


# include <Eigen/Core>
# include <Eigen/Dense>
# include <boost/graph/graph_concepts.hpp>
# include <ros/console.h>
# include <Eigen/StdVector>
# include <ros/ros.h>

namespace itia{
namespace helios{
 
/*
* VelocityProfile(Eigen::VectorXd& qini, unsigned int& np, unsigned int& nfir, double& st);
*/
class VelocityProfile
{
public:

  /*
  * VelocityProfile(Eigen::VectorXd& qini, unsigned int& np, unsigned int& nfir, double& st);
  */
  VelocityProfile(const Eigen::VectorXd& qini, const unsigned int& np, const unsigned int& nfir, const double& st);
  /*
   * addPoints(const Eigen::VectorXd& time, const Eigen::MatrixXd& waypoints, const Eigen::MatrixXd& waypoints_vel, const Eigen::VectorXd& accuracy);
   */
  void addPoints(const Eigen::VectorXd& time, const Eigen::MatrixXd& waypoints, const Eigen::MatrixXd& waypoints_vel, const Eigen::VectorXd& accuracy);
  
  /*
   * addPoints(const Eigen::VectorXd& time, const Eigen::MatrixXd& waypoints, const Eigen::MatrixXd& waypoints_vel, const Eigen::VectorXd& accuracy);
   * return 1 when trajectory is finished
   * return 0 when trajectory is running
   * return -1 when joint limit is reached
   * return -2 when some error occurs
   */
  int update(double& time, Eigen::VectorXd& q, Eigen::VectorXd& Dq, Eigen::VectorXd& DDq);
  void updateVel(const Eigen::VectorXd& input_vel, double& time, Eigen::VectorXd& q, Eigen::VectorXd& Dq, Eigen::VectorXd& DDq);
  void softStop(double& time, Eigen::VectorXd& q, Eigen::VectorXd& Dq, Eigen::VectorXd& DDq);
  
  void changeVelocityScale(const double&  scale);
  
  
  virtual bool setPositionLimit(const Eigen::Ref<const Eigen::VectorXd>& upper_limit, const Eigen::Ref<const Eigen::VectorXd>& lower_limit)
  {
    if ( upper_limit.size() != m_nax)
    {
      ROS_ERROR("upper_limit dimension is wrong");
      return false;
    }
    if ( lower_limit.size() != m_nax)
    {
      ROS_ERROR("upper_limit dimension is wrong");
      return false;
    }
    m_upper_limit=upper_limit;
    m_lower_limit=lower_limit;
    return true;
  }
  
  
protected:
  
  Eigen::MatrixXd m_q_nf;
  Eigen::MatrixXd m_Dq_nf;
  Eigen::MatrixXd m_q;
  Eigen::MatrixXd m_Dq;
  Eigen::MatrixXd m_DDq;
  
  Eigen::MatrixXd m_waypoints;
  Eigen::MatrixXd m_waypoints_vel;
  Eigen::VectorXd m_waypoints_accuracy;
  Eigen::VectorXd m_waypoints_time;
  Eigen::VectorXd m_last_Dq;
  
  
  Eigen::VectorXd m_upper_limit;
  Eigen::VectorXd m_lower_limit;
  double m_t_last_change;
  double m_t_2last_change;
  
  unsigned int m_nfir;
  unsigned int m_np;
  unsigned int m_nax;
  double m_st;
  double m_time;
  bool m_soft_stopped;
  unsigned int m_actual_wp_idx;
  unsigned int m_last_wp_idx;
  
  double m_vel_scale;
  
  void fillBuffer();
  
};



}
}
#endif