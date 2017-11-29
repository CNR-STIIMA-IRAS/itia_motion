
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

#include <itia_helios/helios_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <itia_helios/motion_law_base.h>
#include <itia_helios/mpc_planner.h>

PLUGINLIB_EXPORT_CLASS(itia::helios::HeliosPlannerMPCAction, nodelet::Nodelet) 


namespace itia
{
namespace helios 
{
  
void HeliosPlannerMPCAction::onInit()
{
  m_main_thread  = std::thread(&HeliosPlannerMPCAction::main_thread, this);
  m_stop = false;
  m_namespace = "helios";
}

void HeliosPlannerMPCAction::main_thread()
{
  ros::NodeHandle nh = getNodeHandle();
  m_js_pub = nh.advertise<sensor_msgs::JointState>("joint_target", 1);
  itia::rutils::MsgReceiver<sensor_msgs::JointState> js_rec("/rigid/joint_states");
  ros::Subscriber js_sub = getNodeHandle().subscribe<sensor_msgs::JointState>(
    "/rigid/joint_states", 
    1, 
    &itia::rutils::MsgReceiver<sensor_msgs::JointState>::callback, 
    &js_rec
   );
  
  if (!js_rec.waitForANewData(10))
  {
    ROS_ERROR("'%s' did not received", js_sub.getTopic().c_str());
    stopThreads();
    return;
  }
  
  std::vector<std::string> joint_names;
  if (!nh.getParam("/joint_names", joint_names))
    throw std::invalid_argument("/joint_names does not exist");
  m_nAx = joint_names.size();
  
  if (js_rec.getData().position.size() != m_nAx)
  {
    ROS_ERROR("'%s' and '%s' have different dimensions", js_sub.getTopic().c_str(), (m_namespace+"/joint_names").c_str());
    stopThreads();
    return;
  }
  double st;
  if (!nh.getParam(m_namespace+"/sample_period", st))
    throw std::invalid_argument(m_namespace+"/sample_period does not exist");
  ros::Rate loop_rate(1/st);
  
  m_js_msg.reset(new sensor_msgs::JointState);
  m_js_msg->position.resize(m_nAx);
  m_js_msg->velocity.resize(m_nAx);
  m_js_msg->effort.resize(m_nAx);
  m_js_msg->name.resize(m_nAx);
  Eigen::VectorXd qini(m_nAx);
  qini.setZero();
  itia::JMotion motion(3, m_nAx);
  motion.setZero();
  
  for (int idx = 0;idx<m_nAx;idx++)
  {
    qini(idx) = js_rec.getData().position.at(idx);
    m_js_msg->name.at(idx) = joint_names.at(idx);  
  } 
  itia::helios::MpcPlanner planner(nh, m_namespace, qini);

  bool stop;
  while (ros::ok())
  {
    
    m_stop_mtx.lock();
    stop = m_stop;
    m_stop_mtx.unlock();
    if (stop)
      return;
    
    motion = planner.update();
    for (int iDim = 0;iDim<m_nAx;iDim++)
    {
      m_js_msg->position.at(iDim) = motion(0, iDim);
      m_js_msg->velocity.at(iDim) = motion(1, iDim);
      m_js_msg->effort.at(iDim)   = 0;                      //motion(2, iDim);
    }
    m_js_msg->header.stamp = ros::Time::now();
    m_js_pub.publish(m_js_msg);
    loop_rate.sleep();
  }
  return;
}

HeliosPlannerMPCAction::~HeliosPlannerMPCAction()
{
  ROS_INFO("[HeliosPlannerMPCAction] stop thread");
  stopThreads();
  if (m_main_thread.joinable())
    m_main_thread.join();
  ROS_INFO("[HeliosPlannerMPCAction] thread stopped!");
}

void HeliosPlannerMPCAction::stopThreads()
{
  m_stop_mtx.lock();
  m_stop = true;
  m_stop_mtx.unlock();
};

}
}