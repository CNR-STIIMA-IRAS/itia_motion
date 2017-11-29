
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

#ifndef __MPC_PLANNER__
#define __MPC_PLANNER__



# include <Eigen/Core>
# include <Eigen/Dense>
# include <boost/graph/graph_concepts.hpp>
# include <ros/console.h>
# include <itia_helios/path_base.h>
# include <itia_mutils/quadprog_pc.h>
# include <Eigen/StdVector>
# include <itia_rutils/itia_rutils.h>
# include <stdexcept>
#include <condition_variable>
#include <mutex>
#include <thread>
# include <actionlib/server/simple_action_server.h>
# include <control_msgs/FollowJointTrajectoryAction.h>
# include <itia_motion_msgs/InterpolatePnt.h>
# include <ros/ros.h>

namespace itia{
namespace helios{
  
  class MpcPlanner
  {
  protected:
    
    std::vector<solver::QuadProgPcPtr> m_solvers;
    ros::NodeHandle m_nh;
    boost::shared_ptr<actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>> m_as;
    ros::ServiceServer m_interpolator_server;
    
    double m_st;
    double m_time;
    double m_lambda;
    double m_Dlambda;
    double m_DDlambda;
    unsigned int m_dimension;
    unsigned int m_nu;                                      // n° variables
    unsigned int m_nc;                                      // n° constraints
    unsigned int m_np;                                      // n° prediction
    unsigned int m_nd;                                      // n° derivatives
    unsigned int m_nfir;                                    // n° fir coefficients
    double m_fir_weigth;
    

    std::mutex m_mtx;
    bool m_continue_thread;
    std::condition_variable m_cv;
    std::vector<std::thread> m_threads;
    std::vector<std::unique_ptr<std::mutex>> m_thread_mtx;
    
    Eigen::VectorXd m_time_q;
    Eigen::VectorXd m_qact;
    Eigen::VectorXd m_tollerance;
    Eigen::MatrixXd m_q;
    Eigen::MatrixXd m_scale;
    Eigen::VectorXd m_cruise_jnt_vel;
    
    itia::JMotion motion;
    
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_sp;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_Dsp;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_sp_nofilt;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_Dsp_nofilt;
    
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_h;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_limits;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_x0;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_scale_u;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_scale_constraints;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_bp;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_bm;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_u;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_s;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_lm;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> m_lp;
    
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_F;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_DF;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_L;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_DL;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_A;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_H;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_L_const;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_eye;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_constraints_matrix_limits;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_constraints_matrix_setpoint;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_evoL;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_evoF;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_Fsp;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_FDsp;
    std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>> m_h0;
    
    void computeSp();
    
    void doUpdate(const int unsigned& iDim);
    
    void actionGoalCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
    bool interpolatorCallback(itia_motion_msgs::InterpolatePnt::Request& req, itia_motion_msgs::InterpolatePnt::Response& res);
    
    
  public:
    
    /*
     * MpcPlanner(const ros::NodeHandle& nh, const std::string& parameter_name, const Eigen::Ref<const Eigen::VectorXd>& qini);
     */     
    MpcPlanner(const ros::NodeHandle& nh, const std::string& parameter_name, const Eigen::Ref<const Eigen::VectorXd>& qini);
    
    /*
     * void setNextPoints(const Eigen::Ref<const Eigen::VectorXd>& time, const Eigen::Ref<const Eigen::MatrixXd>& q);
     */     
    void setNextPoints(Eigen::Ref<Eigen::VectorXd> time, const Eigen::Ref<const Eigen::MatrixXd>& q);
    
    /*
     * virtual itia::JMotion&  update();
     */
    virtual itia::JMotion&  update();
    
    ~MpcPlanner()
    {
      m_continue_thread = false;
      m_cv.notify_all();
      for (unsigned int idx = 0;idx<m_dimension;idx++)
        m_threads.at(idx).join();
    }
    
  };
  
  typedef boost::shared_ptr<itia::helios::MpcPlanner> MpcPlannerPtr;
}
}


#endif