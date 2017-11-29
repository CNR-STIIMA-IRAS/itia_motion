
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

#include <itia_fir_planner/itia_fir_planner.h>

namespace itia
{
namespace helios
{

FirPlanner::FirPlanner(const ros::NodeHandle& nh, const std::string& parameter_name, const Eigen::Ref<const Eigen::VectorXd>& qini) 
{
  ROS_INFO("Creating FIR planner");
  m_nh = nh;
  m_diagnostics_pub = m_nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
  m_change_zero_ovr=false;
  
  int tmp;
  if (!nh.getParam(parameter_name+"/dimension", tmp))
  {
    ROS_ERROR("%s", (parameter_name+"/dimension does not exist").c_str());
    throw std::invalid_argument(parameter_name+"/dimension does not exist");
  }
  m_dimension = tmp;
  m_motion.resize(3,m_dimension);  
  m_time = 0;
  m_cruise_jnt_vel.resize(m_dimension);
	
  m_upper_limit.resize(m_dimension);
  m_lower_limit.resize(m_dimension);
  m_upper_limit.setConstant( 100);
  m_lower_limit.setConstant(-100);
  
  
  // LOAD FROM PARAM
  if (!nh.getParam(parameter_name+"/sample_period", m_st))
    throw std::invalid_argument(parameter_name+"/sample_period does not exist");
  
  if (!nh.getParam(parameter_name+"/stop_when_zero_override", m_change_zero_ovr))
  {
    ROS_WARN_STREAM(parameter_name+"/stop_when_zero_override does not exist, set false");
    m_change_zero_ovr=false;
  }
  
  if (!nh.getParam(parameter_name+"/np", tmp))
    throw std::invalid_argument(nh.getNamespace()+"/"+parameter_name+"/np does not exist");
  m_np = tmp;
  
	double acc_vel_ratio;
	if (!nh.getParam(parameter_name+"/acc_vel_ratio", acc_vel_ratio))
		throw std::invalid_argument(parameter_name+"/acc_vel_ratio does not exist");
	m_nfir = int(std::ceil(std::abs(1.0/m_st/acc_vel_ratio)));
	
  std::vector<double> cruise;
  if (!nh.getParam(parameter_name+"/cruise_jnt_vel", cruise))
    throw std::invalid_argument(parameter_name+"/cruise_jnt_vel does not exist");
  
  if (cruise.size() < m_dimension)
    throw std::invalid_argument(parameter_name+"/cruise_jnt_vel has wrong dimension");
  else if (cruise.size() > m_dimension)
    cruise.resize(m_dimension);
  std::copy(cruise.begin(), cruise.end(), m_cruise_jnt_vel.data());
  
  if (m_nfir<2)
    m_nfir = 2;
  
  
  m_last_planned_q =qini;
  m_motion.row(0) = qini;
  
  velocity_profile.reset(new itia::helios::VelocityProfile(m_last_planned_q, m_np, m_nfir, m_st));
  velocity_profile->setPositionLimit(m_upper_limit,m_lower_limit);
  
  m_preempted=false;
  m_continue=true;
//   
  m_as.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(m_nh, "follow_joint_trajectory", 
                                                                                    boost::bind(&itia::helios::FirPlanner::actionGoalCallback,    this,  _1), 
                                                                                    boost::bind(&itia::helios::FirPlanner::actionCancelCallback,  this,  _1), 
                                                                                    false));
  m_as->start();
  m_vel_override=100;
  
  
};

void FirPlanner::ac_thread_function()
{
  
  
  ROS_DEBUG("START ACTION GOAL LOOPING");
  ros::WallRate lp(100);
  while (m_nh.ok() && m_continue)
  {
    lp.sleep();
    if (!m_gh)
    {
      ROS_ERROR("Goal handle is not initialized");
      break;
    }
    if ( (m_preempted) ||  (m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLED) ||  (m_gh->getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED) )
    {
      
      control_msgs::FollowJointTrajectoryResult result;
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).name="INTERPOLATOR";
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::OK;
      if (m_preempted)
        diag_msg.status.at(0).message="preempted";
      else
        diag_msg.status.at(0).message="cancelled";
      diag_msg.status.at(0).values.resize(0);
      m_diagnostics_pub.publish(diag_msg);
      
      m_mtx.lock();
      velocity_profile.reset(new itia::helios::VelocityProfile(m_last_planned_q, m_np, m_nfir, m_st));
      velocity_profile->changeVelocityScale(m_vel_override);
      velocity_profile->setPositionLimit(m_upper_limit,m_lower_limit);
      m_mtx.unlock();
      m_preempted=false;
      ROS_DEBUG("preempted old goal DONE");
      
      break;
    }
    
    
    if (m_is_finished==1)
    {
      control_msgs::FollowJointTrajectoryResult result;
      
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).name="INTERPOLATOR";
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::OK;
      diag_msg.status.at(0).message="all is ok";
      diag_msg.status.at(0).values.resize(0);
      m_diagnostics_pub.publish(diag_msg);
      
      m_gh->setSucceeded(result);
      ROS_DEBUG("EXIT CLEAN");
      break;
    }
    else if (m_is_finished==-1)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code=-4;
      result.error_string="Joint limit reached";
      
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).name="INTERPOLATOR";
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_msg.status.at(0).message="Planning: Joint limit reached";
      diag_msg.status.at(0).values.resize(1+m_dimension*4);
      diag_msg.status.at(0).values.at(0).key="INTERPOLATOR_TYPE";
      diag_msg.status.at(0).values.at(0).value="FIR planner";
      for (unsigned int i=0;i<m_dimension;i++)
      {
        diag_msg.status.at(0).values.at(1+0+(4*i)).key="Joint "+std::to_string(i+1);
        diag_msg.status.at(0).values.at(1+1+(4*i)).key="ACTUAL POSITION";
        diag_msg.status.at(0).values.at(1+2+(4*i)).key="UPPER BOUND";
        diag_msg.status.at(0).values.at(1+3+(4*i)).key="LOWER BOUND";
        diag_msg.status.at(0).values.at(1+0+(4*i)).value = "";
        diag_msg.status.at(0).values.at(1+1+(4*i)).value = std::to_string(m_motion.row(0)(i));
        diag_msg.status.at(0).values.at(1+2+(4*i)).value = std::to_string(m_upper_limit(i));
        diag_msg.status.at(0).values.at(1+3+(4*i)).value = std::to_string(m_lower_limit(i));
      }
      m_diagnostics_pub.publish(diag_msg);
      
      m_gh->setAborted(result);
      break;
    }
    else if (m_is_finished==-2)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code=-4;
      result.error_string="Some problem occurs";
      
      diagnostic_msgs::DiagnosticArray diag_msg;
      diag_msg.header.stamp=ros::Time::now();
      diag_msg.status.resize(1);
      diag_msg.status.at(0).name="INTERPOLATOR";
      diag_msg.status.at(0).hardware_id="FourByThreeRobot";
      diag_msg.status.at(0).level=diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_msg.status.at(0).message="Some problem occurs";
      diag_msg.status.at(0).values.resize(1);
      diag_msg.status.at(0).values.at(0).key="INTERPOLATOR_TYPE";
      diag_msg.status.at(0).values.at(0).value="FIR planner";
      m_diagnostics_pub.publish(diag_msg);
      
      m_gh->setAborted(result);
      break;
    }      
    
  }
  
  m_gh.reset();
  ROS_DEBUG("EXIT");
}

void FirPlanner::changeVelocityOverride(const double& override)
{
  m_vel_override=override;
  if (velocity_profile)
    velocity_profile->changeVelocityScale(m_vel_override);
  
}

itia::JMotion&  FirPlanner::update()
{
  
  double time;
  Eigen::VectorXd Dqact(m_dimension);
  Eigen::VectorXd DDqact(m_dimension);
  
  m_mtx.lock();
  m_is_finished = velocity_profile->update(time, m_last_planned_q, Dqact, DDqact);
  m_mtx.unlock();
  
  if ((m_vel_override==0) && (m_change_zero_ovr) )
  {
    if (m_gh)
    {
      m_gh->setCanceled();
      if (m_as_thread.joinable())
        m_as_thread.join();
      m_gh.reset();
    }
  }
  
  m_motion.row(0)=m_last_planned_q.transpose();
  m_motion.row(1)=Dqact.transpose();
  m_motion.row(2)=DDqact.transpose();
  return m_motion;
};

JMotion& FirPlanner::updateVel( const Eigen::Ref<const Eigen::VectorXd>& vel)
{
  double time;
  Eigen::VectorXd Dqact(m_dimension);
  Eigen::VectorXd DDqact(m_dimension);
  m_is_finished = false;
  
  m_mtx.lock();
  velocity_profile->updateVel(vel,time, m_last_planned_q, Dqact, DDqact);
  m_mtx.unlock();
  
  m_motion.row(0)=m_last_planned_q.transpose();
  m_motion.row(1)=Dqact.transpose();
  m_motion.row(2)=DDqact.transpose();
  return m_motion;
  
}

JMotion& FirPlanner::softStop()
{
  double time;
  Eigen::VectorXd Dqact(m_dimension);
  Eigen::VectorXd DDqact(m_dimension);
  m_is_finished = false;
  
  m_mtx.lock();
  velocity_profile->softStop(time, m_last_planned_q, Dqact, DDqact);
  m_mtx.unlock();
  
  m_motion.row(0)=m_last_planned_q.transpose();
  m_motion.row(1)=Dqact.transpose();
  m_motion.row(2)=DDqact.transpose();
  return m_motion;
}
  
void FirPlanner::actionGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle gh)
{
  ROS_DEBUG("received a goal");
  //control_msgs::FollowJointTrajectoryGoalPtr goal=gh.getGoal();
  auto goal=gh.getGoal();
  
  
  boost::shared_ptr<actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle> current_gh;
  
  if (m_gh)
  {
    // PREEMPTED OLD GOAL
    
    ROS_DEBUG("preempting old goal");
    m_gh->setAborted();
    m_preempted=true;
    
    if (m_as_thread.joinable())
      m_as_thread.join();
    ROS_DEBUG("STOPPED");
    m_preempted=false;
  }
  else
  {
    ROS_DEBUG("No goals running yet");
  }
  
  current_gh.reset(new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle(gh));
  m_gh=current_gh;
  unsigned  int nPnt = goal->trajectory.points.size();
  if (nPnt==0)
  {
    ROS_DEBUG("TRAJECTORY WITH NO POINT");
    control_msgs::FollowJointTrajectoryResult result;
    m_gh->setAccepted();
    current_gh->setSucceeded(result);
    m_gh.reset();
    return;
  }
  
  ROS_DEBUG("NUMBER OF POINT =%d",nPnt);  
  Eigen::MatrixXd waypoints(nPnt, m_dimension);
  Eigen::MatrixXd waypoints_vel(nPnt, m_dimension);
  Eigen::VectorXd waypoints_accuracy(nPnt, 1);
  Eigen::VectorXd time(nPnt, 1);
  
  double path_accuracy;
  if (goal->path_tolerance.size()>0)
    path_accuracy = goal->path_tolerance.at(0).position;
  else
    if (!m_nh.getParam("fir_params/default_path_accuracy", path_accuracy))
    {
      ROS_WARN("PARAMETER 'fir_params/default_path_accuracy' DOES NOT EXIST,  set default value equal to 0.0");
      path_accuracy = 0.0;
    }
  
  waypoints_accuracy.setConstant(path_accuracy);
  
  double goal_accuracy;
  if (goal->goal_tolerance.size()>0)
    goal_accuracy = goal->goal_tolerance.at(0).position;
  else
    if (!m_nh.getParam("fir_params/fir_params/default_goal_accuracy", goal_accuracy))
    {
      ROS_DEBUG("PARAMETER '%s/fir_params/default_goal_accuracy' DOES NOT EXIST,  set default value equal to 0.0",m_nh.getNamespace().c_str());
      goal_accuracy = 0.0;
    }
  waypoints_accuracy(waypoints_accuracy.rows()-1) = goal_accuracy;
  
  bool use_time=false;
  if (!m_nh.getParam("fir_params/use_time", use_time))
  {
    ROS_DEBUG("fir_params/use_time is not set, use FALSE");
    use_time=false;
  }
  for (unsigned int iPnt = 0;iPnt<nPnt;iPnt++)
  {
    for (unsigned int iDim = 0;iDim<m_dimension;iDim++)
    {
      waypoints(iPnt, iDim) = goal->trajectory.points.at(iPnt).positions.at(iDim);
      waypoints_vel(iPnt, iDim) = m_cruise_jnt_vel(iDim);
    }
    if (use_time)
      time(iPnt) = goal->trajectory.points.at(iPnt).time_from_start.toSec();
    else 
      time(iPnt)=0;
  } 
    
  
  m_mtx.lock();
  velocity_profile.reset(new itia::helios::VelocityProfile(m_last_planned_q, m_np, m_nfir, m_st));
  velocity_profile->changeVelocityScale(m_vel_override);
  velocity_profile->setPositionLimit(m_upper_limit,m_lower_limit);
  velocity_profile->addPoints(time, waypoints.transpose(), waypoints_vel.transpose(), waypoints_accuracy);
  m_mtx.unlock();
  
  ROS_DEBUG("Starting managing new goal");
  m_gh->setAccepted();
  
  m_is_finished=0;
  if (m_as_thread.joinable())
    m_as_thread.join();
  
  m_as_thread = std::thread(&FirPlanner::ac_thread_function,this);

  ROS_DEBUG("EXIT");
}


void FirPlanner::actionCancelCallback(actionlib::ActionServer< control_msgs::FollowJointTrajectoryAction >::GoalHandle gh)
{
  ROS_DEBUG("Stopping active goal");
  if (m_gh)
  {
    m_gh->setCanceled();
    if (m_as_thread.joinable())
      m_as_thread.join();
    m_gh.reset();
  }
  else
    ROS_WARN("no goal to cancel");
  
  

}


}
  
}
