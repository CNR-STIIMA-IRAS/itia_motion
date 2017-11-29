#include <itia_fir_planner/itia_fir_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <itia_rutils/itia_rutils.h>
#include <itia_fir_planner/itia_fir_interpolate_service.h>
#include <itia_motion_msgs/InterpolatePnt.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interpolate_trj_server");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  
  ros::ServiceServer interpolate_trj = nh.advertiseService("interpolate_trj", &itia::helios::interpolateTrajectories);
  ROS_INFO("%sTRAJECTORY INTERPOLATION SERVER%s", BOLDGREEN, RESET);
  ROS_INFO("starting service '%s%s%s'", BOLDGREEN, interpolate_trj.getService().c_str(), RESET);
  
  ros::spin();
}