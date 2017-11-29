
 // -------------------------------------------------------------------------------- 
 // Copyright (c) 2017 CNR-ITIA <iras@itia.cnr.it>
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions are met:
 //
 // 1. Redistributions of source code must retain the above copyright notice,
 // this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 // notice, this list of conditions and the following disclaimer in the
 // documentation and/or other materials provided with the distribution.
 // 3. Neither the name of mosquitto nor the names of its
 // contributors may be used to endorse or promote products derived from
 // this software without specific prior written permission.
 //
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 // POSSIBILITY OF SUCH DAMAGE.
 // -------------------------------------------------------------------------------- 

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