
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

#ifndef __ITIA_FIR_PLANNER__
#define __ITIA_FIR_PLANNER__



# include <Eigen/Core>
# include <Eigen/Dense>
# include <boost/graph/graph_concepts.hpp>
# include <ros/console.h>
# include <Eigen/StdVector>
# include <itia_rutils/itia_rutils.h>
# include <stdexcept>
# include <condition_variable>
# include <mutex>
# include <thread>
# include <actionlib/server/simple_action_server.h>
# include <control_msgs/FollowJointTrajectoryAction.h>
# include <itia_dynamics_core/itia_primitives.h>
# include <ros/ros.h>
# include <itia_fir_planner/velocity_profile.h>
# include <diagnostic_msgs/DiagnosticArray.h>

namespace itia{
namespace helios{
  
  class FirPlanner
  {
  public:
    FirPlanner(const ros::NodeHandle& nh, const std::string& parameter_name, const Eigen::Ref<const Eigen::VectorXd>& qini);
    
  
    void setNextPoints(Eigen::Ref<Eigen::VectorXd> time, const Eigen::Ref<const Eigen::MatrixXd>& q);
    
    /*
     * virtual itia::JMotion&  update();
     */
    virtual itia::JMotion&  update();
    
    virtual itia::JMotion& updateVel( const Eigen::Ref<const Eigen::VectorXd>& vel);
    
    virtual itia::JMotion& softStop( );
    
    void changeVelocityOverride(const double&  override);
    
    ~FirPlanner()
    {
      m_continue=false;
      if (m_as_thread.joinable())
        m_as_thread.join();
    }
    
    
    virtual bool setPositionLimit(const Eigen::Ref<const Eigen::VectorXd>& upper_limit, const Eigen::Ref<const Eigen::VectorXd>& lower_limit)
    {
      m_lower_limit=lower_limit;
      m_upper_limit=upper_limit;
      if (!velocity_profile)
        return false;
      
      return velocity_profile->setPositionLimit(upper_limit,lower_limit);
    }

  protected:
    ros::NodeHandle m_nh;
    boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>> m_as;
    boost::shared_ptr<itia::helios::VelocityProfile> velocity_profile;
    void actionGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
    //void actionCancelCallback(const    control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
    void actionCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh);
    
    std::thread m_as_thread;
    void ac_thread_function();
    
    ros::Publisher m_diagnostics_pub;
    boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle> m_gh;
    
    double m_st;
    double m_time;
    unsigned int m_nfir;                                    // n° fir coefficients
    unsigned int m_np;                                      // n° prediction
    double m_vel_override;
    
    unsigned int m_dimension;
    int m_is_finished;
    bool m_continue;
    bool m_preempted;
    bool m_change_zero_ovr;
    itia::JMotion m_motion;
    
    Eigen::VectorXd m_cruise_jnt_vel;
    Eigen::VectorXd m_last_planned_q;
    
    Eigen::VectorXd m_upper_limit;
    Eigen::VectorXd m_lower_limit;

    
    std::mutex m_mtx;
  };
  
  typedef boost::shared_ptr<itia::helios::FirPlanner> FirPlannerPtr;
}
}


#endif