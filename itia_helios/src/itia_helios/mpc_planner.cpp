
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

#include <itia_helios/mpc_planner.h>


namespace itia
{
namespace helios
{

/*
  * MpcPlanner(const ros::NodeHandle& nh, const std::string& parameter_name, const Eigen::Ref<const Eigen::VectorXd>& qini);
  */     
MpcPlanner::MpcPlanner(const ros::NodeHandle& nh, const std::string& parameter_name, const Eigen::Ref< const Eigen::VectorXd >& qini)
{
  ROS_INFO("Creating MPC planner");
  m_nh = nh;
  m_continue_thread = true;
  int tmp;
  if (!nh.getParam(parameter_name+"/dimension", tmp))
  {
    ROS_FATAL("%s", (parameter_name+"/dimension does not exist").c_str());
    throw std::invalid_argument(parameter_name+"/dimension does not exist");
  }
  m_dimension = tmp;
  ROS_INFO("Creating MPC planner");
  
  m_time = 0;
  m_time_q.resize(1);
  m_scale.resize(1, m_dimension);
  
  m_q.resize(1, m_dimension);
  m_q = qini.transpose();
  m_qact.resize(m_dimension);
  m_qact = qini;
  m_tollerance.resize(m_dimension);
  m_tollerance.setConstant(1e-3);
  m_time_q(0) =0;
  m_scale.setOnes();
  m_cruise_jnt_vel.resize(m_dimension);
  
  m_solvers.resize(m_dimension);

  // LOAD FROM PARAM
  
  if (!nh.getParam(parameter_name+"/sample_period", m_st))
    throw std::invalid_argument(parameter_name+"/sample_period does not exist");
  
  if (!nh.getParam(parameter_name+"/lambda", m_lambda))
    throw std::invalid_argument(parameter_name+"/lambda does not exist");
  if (!nh.getParam(parameter_name+"/Dlambda", m_Dlambda))
    throw std::invalid_argument(parameter_name+"/Dlambda does not exist");
  if (!nh.getParam(parameter_name+"/DDlambda", m_DDlambda))
    throw std::invalid_argument(parameter_name+"/DDlambda does not exist");
  
  if (!nh.getParam(parameter_name+"/nu", tmp))
    throw std::invalid_argument(parameter_name+"/nu does not exist");
  m_nu = tmp;
  
  if (!nh.getParam(parameter_name+"/nfir", tmp))
    throw std::invalid_argument(parameter_name+"/nfir does not exist");
  m_nfir = tmp;
  if (m_nfir<2)
    m_nfir = 2;
  m_fir_weigth = 1.0/m_nfir;
  
  if (!nh.getParam(parameter_name+"/nc", tmp))
    throw std::invalid_argument(parameter_name+"/nc does not exist");
  m_nc = tmp;

  if (!nh.getParam(parameter_name+"/np", tmp))
    throw std::invalid_argument(parameter_name+"/np does not exist");
  m_np = tmp;
  
  if (!nh.getParam(parameter_name+"/nd", tmp))
    throw std::invalid_argument(parameter_name+"/nd does not exist");
  m_nd = tmp;
  
//   # pragma message("MANAGE 'interpolate_trj' service")
//   m_interpolator_server = nh.advertiseService<itia_motion_msgs::InterpolatePnt>("interpolate_trj", &itia::helios::MpcPlanner::interpolatorCallback, this);
  
  m_sp.resize(m_dimension);
  m_Dsp.resize(m_dimension);
  m_sp_nofilt.resize(m_dimension);
  m_Dsp_nofilt.resize(m_dimension);
  m_h.resize(m_dimension);
  m_limits.resize(m_dimension);
  m_x0.resize(m_dimension);
  m_scale_u.resize(m_dimension);
  m_scale_constraints.resize(m_dimension);
  m_bp.resize(m_dimension);
  m_bm.resize(m_dimension);
  m_u.resize(m_dimension);
  m_s.resize(m_dimension);
  m_lm.resize(m_dimension);
  m_lp.resize(m_dimension);
  m_F.resize(m_dimension);
  m_DF.resize(m_dimension);
  m_L.resize(m_dimension);
  m_L_const.resize(m_dimension);
  m_DL.resize(m_dimension);
  m_eye.resize(m_dimension);
  m_constraints_matrix_limits.resize(m_dimension);
  m_constraints_matrix_setpoint.resize(m_dimension);
  m_evoF.resize(m_dimension);
  m_evoL.resize(m_dimension);
  m_FDsp.resize(m_dimension);
  m_Fsp.resize(m_dimension);
  m_h0.resize(m_dimension);
  m_A.resize(m_dimension);
  m_H.resize(m_dimension);
  m_threads.resize(m_dimension);
  m_thread_mtx.resize(m_dimension);
 
  {
    Eigen::MatrixXd tmp_matrix;
    if (!itia::rutils::getParam(m_nh, parameter_name+"/cruise_joint_vel", tmp_matrix))
      throw std::invalid_argument(parameter_name+"/cruise_joint_vel does not exist");
    m_cruise_jnt_vel = tmp_matrix;
  }
  for (unsigned int idx = 0;idx<m_dimension;idx++)
  {
    m_thread_mtx.at(idx).reset(new std::mutex);
    m_sp.at(idx).resize(m_np);
    m_Dsp.at(idx).resize(m_np);
    m_sp_nofilt.at(idx).resize(m_np+m_nfir-1);
    m_Dsp_nofilt.at(idx).resize(m_np+m_nfir-1);
    m_sp_nofilt.at(idx).setConstant(m_qact(idx));
    m_Dsp_nofilt.at(idx).setConstant(0.0);
    
    m_h.at(idx).resize(m_nu);
    m_limits.at(idx).resize(m_nd);
    m_x0.at(idx).resize(m_nd-1);
    m_scale_u.at(idx).resize(m_nu);
    m_scale_constraints.at(idx).resize(m_nc);
    m_A.at(idx).resize(m_nu, m_nc); 
    m_bp.at(idx).resize(m_nc);
    m_bm.at(idx).resize(m_nc);
    m_u.at(idx).resize(m_nu);
    m_s.at(idx).resize(m_nc);
    m_lp.at(idx).resize(m_nc);
    m_lm.at(idx).resize(m_nc);
    m_F.at(idx).resize(m_np, m_nu);
    m_L.at(idx).resize(m_np, m_nd-1); 
    m_DF.at(idx).resize(m_np, m_nu);
    m_DL.at(idx).resize(m_np, m_nd-1); 
    m_eye.at(idx).resize(m_nu, m_nu);
    m_constraints_matrix_limits.at(idx).resize(m_nc, m_nd);
    m_constraints_matrix_setpoint.at(idx).resize(m_nc, m_np);
    m_evoF.at(idx).resize(m_nd-1);
    m_evoL.at(idx).resize(m_nd-1, m_nd-1);
    
    
    m_x0.at(idx).setZero();
    m_x0.at(idx)(0) = qini(idx);
    m_u.at(idx).setZero();
    m_s.at(idx).setZero();
    m_lp.at(idx).setConstant(1e-5);
    m_lm.at(idx).setConstant(1e-5);
    m_eye.at(idx).setIdentity();
    
    
    Eigen::MatrixXd tmp_matrix;
    if (!itia::rutils::getParam(m_nh, parameter_name+"/limits_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/limits_"+std::to_string(idx)+" does not exist");
    
    m_limits.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/scale_u_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/scale_u_"+std::to_string(idx)+" does not exist");
    m_scale_u.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/scale_constraints_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/scale_constraints_"+std::to_string(idx)+" does not exist");
    m_scale_constraints.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/F_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/F_"+std::to_string(idx)+" does not exist");
    m_F.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/DF_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/DF_"+std::to_string(idx)+" does not exist");
    m_DF.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/L_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/L_"+std::to_string(idx)+" does not exist");
    m_L.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/L_constraints_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/L_constraints_"+std::to_string(idx)+" does not exist");
    m_L_const.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/DL_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/DL_"+std::to_string(idx)+" does not exist");
    m_DL.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/A_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/A_"+std::to_string(idx)+" does not exist");
    m_A.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/constraints_matrix_limits_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/constraints_matrix_limits_"+std::to_string(idx)+" does not exist");
    m_constraints_matrix_limits.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/constraints_matrix_sp_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/constraints_matrix_sp_"+std::to_string(idx)+" does not exist");
    m_constraints_matrix_setpoint.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/evo_L_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/evo_L_"+std::to_string(idx)+" does not exist");
    m_evoL.at(idx) =tmp_matrix;
    
    if (!itia::rutils::getParam(m_nh, parameter_name+"/evo_F_"+std::to_string(idx), tmp_matrix))
      throw std::invalid_argument(parameter_name+"/evo_F_"+std::to_string(idx)+" does not exist");
    m_evoF.at(idx) =tmp_matrix;
    
    m_Fsp.at(idx) = -m_lambda*m_F.at(idx).transpose();
    m_FDsp.at(idx) = - m_Dlambda*m_DF.at(idx).transpose();
    m_h0.at(idx) = m_lambda*m_F.at(idx).transpose() * m_L.at(idx) + m_Dlambda*m_F.at(idx).transpose() * m_DL.at(idx);

    m_H.at(idx) = m_lambda*m_F.at(idx).transpose() *m_F.at(idx)+
                  m_Dlambda*m_DF.at(idx).transpose() *m_DF.at(idx)+
                  m_DDlambda*m_eye.at(idx);
    m_solvers.at(idx).reset(new solver::QuadProgPc(m_H.at(idx), m_A.at(idx)));
    
   
  }
  
  motion.resize(m_nd, m_dimension);
  motion.setZero();
  motion.row(0) = m_qact;
  
  
  computeSp();
    
  for (unsigned int idx = 0;idx<m_dimension;idx++)
    m_threads.at(idx) = std::thread(std::bind(&itia::helios::MpcPlanner::doUpdate,this, idx));
  
  m_as.reset(new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>(m_nh, "follow_joint_trajectory", 
  boost::bind(&itia::helios::MpcPlanner::actionGoalCallback,  this,  _1), false));
  
  m_as->start();
}

void MpcPlanner::computeSp()
{
  
  for (unsigned int iDim = 0;iDim<m_dimension;iDim++)
  {
    for (unsigned int iCy = 0;iCy<m_np;iCy++)
    {
      m_sp.at(iDim)(iCy) = m_sp_nofilt.at(iDim).block(iCy, 0, m_nfir, 1).mean();
      m_Dsp.at(iDim)(iCy) = m_Dsp_nofilt.at(iDim).block(iCy, 0, m_nfir, 1).mean();
    }
    m_sp_nofilt.at(iDim).block(0, 0, m_sp_nofilt.at(iDim).rows()-1, 1) = m_sp_nofilt.at(iDim).block(1, 0, m_sp_nofilt.at(iDim).rows()-1, 1);
    m_Dsp_nofilt.at(iDim).block(0, 0, m_Dsp_nofilt.at(iDim).rows()-1, 1) = m_Dsp_nofilt.at(iDim).block(1, 0, m_Dsp_nofilt.at(iDim).rows()-1, 1);
  } 
  
  unsigned int iCy = m_np-1;
  unsigned int iSp = 0;
  
  while ((m_time+(iCy+1)*m_st) > m_time_q(iSp))
  {
    if (iSp >= (m_time_q.rows()-1))
      break;
    else
      iSp++;
  }
  
  Eigen::VectorXd velocity(m_dimension);
  double remaining_time = std::max(m_time_q(iSp)-(m_time+(iCy)*m_st), m_st);
  if (iSp>0 && iSp<(m_time_q.rows()-1))
    if ((m_q.row(iSp)-m_q.row(iSp-1)).norm() <1e-5)
      remaining_time = m_st;
  for (unsigned int iDim = 0;iDim<m_dimension;iDim++)
  {
//       velocity(iDim) = std::abs(m_sp_nofilt.at(iDim)(iCy+m_nfir-2) - m_q(iSp, iDim))/m_st;
    velocity(iDim) = std::abs(m_sp_nofilt.at(iDim)(iCy+m_nfir-2) - m_q(iSp, iDim))/remaining_time;
  }
  double scale = std::max(1.0, (velocity.array()/m_cruise_jnt_vel.array()).maxCoeff());
  // Compute new element
  for (unsigned int iDim = 0;iDim<m_dimension;iDim++)
  {
    double vel = velocity(iDim)/scale;
    
    if (m_sp_nofilt.at(iDim)(iCy+m_nfir-2) < (m_q(iSp, iDim)-vel*m_st))
      m_Dsp_nofilt.at(iDim)(iCy+m_nfir-1) = vel;
    else if (m_sp_nofilt.at(iDim)(iCy+m_nfir-2) > (m_q(iSp, iDim)+vel*m_st))
      m_Dsp_nofilt.at(iDim)(iCy+m_nfir-1) = -vel;
    else
      m_Dsp_nofilt.at(iDim)(iCy+m_nfir-1) = ( m_q(iSp, iDim) - m_sp_nofilt.at(iDim)(iCy+m_nfir-2) )/m_st;
    m_sp_nofilt.at(iDim)(iCy+m_nfir-1) = m_sp_nofilt.at(iDim)(iCy+m_nfir-2)+ m_Dsp_nofilt.at(iDim)(iCy+m_nfir-1)*m_st;
  } 
  
    
}

void MpcPlanner::setNextPoints(Eigen::Ref<Eigen::VectorXd> time, const Eigen::Ref<const Eigen::MatrixXd>& q)
{
  // TODO Check if time > m_time,  manage append
  m_q = q;
  m_time_q = time.array()+m_time;

  for (unsigned int iCy = 0;iCy<m_np;iCy++ )
  {
    unsigned int iSp = 0;
    
    while ((m_time+(iCy+1)*m_st) > m_time_q(iSp))
    {
      if (iSp >= (m_time_q.rows()-1))
        break;
      else
        iSp++;
    }
    
    Eigen::VectorXd velocity(m_dimension);
    double remaining_time = std::max(m_time_q(iSp)-(m_time+(iCy)*m_st), m_st);
    if (iSp>0 && iSp<(m_time_q.rows()-1))
      if ((m_q.row(iSp)-m_q.row(iSp-1)).norm() <1e-5)
      remaining_time = m_st;
    for (unsigned int iDim = 0;iDim<m_dimension;iDim++)
    {
      velocity(iDim) = std::abs(m_sp_nofilt.at(iDim)(iCy+m_nfir-2) - m_q(iSp, iDim))/remaining_time;
    }
    double scale = std::max(1.0, (velocity.array()/m_cruise_jnt_vel.array()).maxCoeff());
    
    
    for (unsigned int iDim = 0;iDim<m_dimension;iDim++)
    {
      double vel = velocity(iDim)/scale;
      if (m_sp_nofilt.at(iDim)(iCy+m_nfir-2) < (m_q(iSp, iDim)-vel*m_st))
        m_Dsp_nofilt.at(iDim)(iCy+m_nfir-1) = vel;
      else if (m_sp_nofilt.at(iDim)(iCy+m_nfir-2) > (m_q(iSp, iDim)+vel*m_st))
        m_Dsp_nofilt.at(iDim)(iCy+m_nfir-1) = -vel;
      else
        m_Dsp_nofilt.at(iDim)(iCy+m_nfir-1) = ( m_q(iSp, iDim) - m_sp_nofilt.at(iDim)(iCy+m_nfir-2) )/m_st;
      m_sp_nofilt.at(iDim)(iCy+m_nfir-1) = m_sp_nofilt.at(iDim)(iCy+m_nfir-2)+ m_Dsp_nofilt.at(iDim)(iCy+m_nfir-1)*m_st;
    } 
    
  }

}


void MpcPlanner::doUpdate(const unsigned int& iDim)
{
  while (m_continue_thread)
  {
    m_thread_mtx.at(iDim)->lock();
    m_bm.at(iDim) = -m_constraints_matrix_limits.at(iDim)*m_limits.at(iDim)-m_L_const.at(iDim) *m_x0.at(iDim)+m_constraints_matrix_setpoint.at(iDim) *m_sp.at(iDim); 
    m_bp.at(iDim) =  m_constraints_matrix_limits.at(iDim)*m_limits.at(iDim)-m_L_const.at(iDim) *m_x0.at(iDim)+m_constraints_matrix_setpoint.at(iDim) *m_sp.at(iDim); 
    m_h.at(iDim) = m_Fsp.at(iDim) *m_sp.at(iDim) + m_FDsp.at(iDim) *m_Dsp.at(iDim) + m_h0.at(iDim) *m_x0.at(iDim);
    
    m_solvers.at(iDim)->solve(m_h.at(iDim), m_bm.at(iDim), m_bp.at(iDim), m_u.at(iDim), m_s.at(iDim), m_lm.at(iDim), m_lp.at(iDim));
    m_x0.at(iDim) = m_evoF.at(iDim)*m_u.at(iDim)(0)+m_evoL.at(iDim)*m_x0.at(iDim);
    motion.col(iDim) <<  m_x0.at(iDim)(0), m_x0.at(iDim)(1), 0;               //m_u.at(iDim)(0) *m_scale_constraints.at(iDim)(0);
    m_qact = motion.row(0);
    
    m_thread_mtx.at(iDim)->unlock();
    
    std::unique_lock<std::mutex> lk(m_mtx);
    m_cv.wait(lk);
  }
}

itia::JMotion&  MpcPlanner::update()
{
  computeSp();
  m_cv.notify_all();
  for (unsigned int iDim = 0;iDim<m_dimension;iDim++)
  {
    m_thread_mtx.at(iDim)->lock();
    m_thread_mtx.at(iDim)->unlock();
  }
  m_time += m_st;
  m_qact = motion.row(0);
  return motion;
};

void MpcPlanner::actionGoalCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
{
  
  ROS_DEBUG("received a goal");
  unsigned int nPnt = goal->trajectory.points.size();
//   ROS_INFO("received %d points", nPnt);
    
  Eigen::MatrixXd q(nPnt, m_dimension);
  Eigen::VectorXd time(nPnt);
  
  trajectory_msgs::JointTrajectoryPoint pnt;
  for (unsigned int iPnt = 0; iPnt < nPnt; iPnt++)
  {
    for (unsigned int iDim = 0; iDim<m_dimension;iDim++)
    {
      q(iPnt, iDim) = goal->trajectory.points.at(iPnt).positions.at(iDim);
    }
    time(iPnt) = goal->trajectory.points.at(iPnt).time_from_start.toSec();
  };
  
  setNextPoints(time, q);
  
  if (goal->goal_tolerance.size() == 1)
    m_tollerance.setConstant(goal->goal_tolerance.at(0).position);
  else if (goal->goal_tolerance.size() == 0)
    m_tollerance.setConstant(1e-3);
  else if (goal->goal_tolerance.size() == m_dimension)
    for (unsigned int idx = 0;idx<m_dimension;idx++)
      m_tollerance(idx) = goal->goal_tolerance.at(idx).position;
  else 
  {
    ROS_ERROR("Goal tolerance has wrong dimension");
    m_tollerance.setConstant(1e-3);
  }
  
  ros::Time t0 = ros::Time::now();
  while (m_continue_thread)
  {
    
    if (m_as->isPreemptRequested())
    {
      ROS_DEBUG("HELIOS PREEMPTED");
      if (m_as->isNewGoalAvailable())
      {
        control_msgs::FollowJointTrajectoryResult result;
        m_as->setPreempted(result);
        return;
      }
      ROS_DEBUG("position set equal to:");
      time.resize(1);
      time(0) = 0;
      std::cout << "time = " <<  time(0) << ",  q = " << m_qact.transpose() <<  std::endl;
      
      setNextPoints(time, m_qact.transpose());
      break;
    }
    ros::Duration(m_st).sleep();
    if ((ros::Time::now()-t0).toSec() < time(time.cols()-1))
      continue;
    Eigen::VectorXd error = (q.bottomRows(1).transpose()-m_qact).array().abs();
    bool in_tollerance = true;
    for (int idx = 0;idx<error.size();idx++)
    {
      if (error(idx)>m_tollerance(idx))
      {
        in_tollerance = false;
      }
    }
    if (in_tollerance)
      break;
  }
//   ROS_INFO("movement finished in %e [s]", (ros::Time::now()-t0).toSec());
//   ros::Duration(10).sleep();
//   ROS_INFO("movement finished in %e [s]", (ros::Time::now()-t0).toSec());
  control_msgs::FollowJointTrajectoryResult result;
  m_as->setSucceeded(result);
}

bool MpcPlanner::interpolatorCallback(itia_motion_msgs::InterpolatePnt::Request& req, itia_motion_msgs::InterpolatePnt::Response& res)
{
//   res.trajectory_full = req.trajectory_viapoints;
  return true;
}


}
  
}