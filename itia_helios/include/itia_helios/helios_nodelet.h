#ifndef __HELIOS_NODELET__
#define __HELIOS_NODELET__

# include <nodelet/nodelet.h>
# include <thread>
# include <mutex>
#include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <itia_rutils/itia_rutils.h>
# include <sensor_msgs/JointState.h>

namespace itia
{
namespace helios 
{
  
  class HeliosPlannerMPCAction : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  protected:
    std::thread m_main_thread;
    std::mutex m_stop_mtx;
    bool m_stop;
    int m_nAx;
    std::string m_namespace;
    bool m_received_fb;
    
    ros::Publisher m_js_pub;
    sensor_msgs::JointStatePtr m_js_msg;
    
    void main_thread();
    
    void timerCallback(const ros::TimerEvent& event);
    void stopThreads();
    ~HeliosPlannerMPCAction();
    
    void fbCallback(const sensor_msgs::JointStateConstPtr& msg);
  };

}
}

# endif