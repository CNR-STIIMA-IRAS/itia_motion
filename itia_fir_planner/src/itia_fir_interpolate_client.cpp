#include <itia_fir_planner/itia_fir_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <itia_rutils/itia_rutils.h>
#include <itia_fir_planner/velocity_profile.h>
#include <itia_motion_msgs/InterpolatePnt.h>


bool getTrajectoryFromParam(const ros::NodeHandle& nh, const std::string& trj_name, trajectory_msgs::JointTrajectory& trj)
{
  std::vector<std::vector<double>> positions;
  if (!itia::rutils::getParamMatrix(nh, trj_name+"/positions", positions))
  {
    ROS_ERROR("%s/positions does not exist", trj_name.c_str());
    return false;
  }
  int npnt = positions.size();
  if (npnt == 0)
  {
    ROS_ERROR("%s/positions with no points", trj_name.c_str());
    return -1;
  }
  int dimension = positions.at(0).size();
  if (npnt == 0)
  {
    ROS_ERROR("%s/positions with no dimensions", trj_name.c_str());
    return false;
  }
  
  std::vector<double> time;
  if (!nh.getParam(trj_name+"/time_from_start", time))
  {
    ROS_ERROR("%s/time_from_start does not exist", trj_name.c_str());
    return false;
  }
  if (time.size() != npnt)
  {
    ROS_ERROR("%s/time_from_start has wrong dimensions", trj_name.c_str());
    return false;
  }
  trj.points.resize(npnt);
  
  for (int iPnt = 0;iPnt<npnt;iPnt++)
  {
    trj.points.at(iPnt).positions.resize(dimension);
    trj.points.at(iPnt).positions = positions.at(iPnt);
    trj.points.at(iPnt).time_from_start = ros::Duration(time.at(iPnt));
  }
 return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interpolate_trj_client");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  
  trajectory_msgs::JointTrajectory trajectory_waypoints;
  trajectory_msgs::JointTrajectory trajectory_final;

  if (!getTrajectoryFromParam(nh, "waypoints_trajectory", trajectory_waypoints))
    return -1;
  
  control_msgs::JointTolerance path_tolerance;
  control_msgs::JointTolerance goal_tolerance;
  if (!nh.getParam("path_tolerance",path_tolerance.position))
    path_tolerance.position = 0.001;
  if (!nh.getParam("goal_tolerance",goal_tolerance.position))
    goal_tolerance.position = 0.00;

  ros::ServiceClient interpolate_trj_src_client = nh.serviceClient<itia_motion_msgs::InterpolatePnt>("interpolate_trj");
  
  ROS_INFO("%sTRAJECTORY INTERPOLATION CLIENT EXAMPLE%s", BOLDGREEN, RESET);
  ROS_INFO("waiting for service existance '%s%s%s'", BOLDGREEN, interpolate_trj_src_client.getService().c_str(), RESET);
  
  interpolate_trj_src_client.waitForExistence();
  ROS_INFO("service '%s%s%s' exists", BOLDGREEN, interpolate_trj_src_client.getService().c_str(), RESET);
  
  itia_motion_msgs::InterpolatePnt srv;
  
  
  srv.request.trajectory_waypoints = trajectory_waypoints;
  srv.request.max_speed = 1;
  srv.request.delta_time = 0.001;
  srv.request.goal_tolerance.push_back(goal_tolerance);
  srv.request.path_tolerance.push_back(path_tolerance);
  
  if (!nh.getParam("delta_time",srv.request.delta_time))
    srv.request.delta_time = 0.001;
  if (!nh.getParam("max_velocity",srv.request.max_speed))
    srv.request.max_speed = 1.0;
  
  
  ROS_INFO("Calling service server");
  ros::Time tc=ros::Time::now();
  if (!interpolate_trj_src_client.call(srv))
  {
    ROS_ERROR("  FAILED");
    return 0;
  }
  ROS_INFO("%scorrect result%s", BOLDGREEN, RESET);
  ROS_INFO("computational time=%s%f [s]%s", BOLDGREEN,(ros::Time::now()-tc).toSec(),RESET);
  trajectory_final = srv.response.trajectory_full;
  
  ROS_INFO("resulting trajectory has %s%zu%s points", BOLDGREEN, trajectory_final.points.size(), RESET);
  return 0;
  
  sensor_msgs::JointState msg;
  int dimension = trajectory_final.points.at(0).positions.size();
  msg.position.resize(dimension);
  msg.velocity.resize(dimension);
  msg.effort.resize(dimension);
  std::fill(msg.position.begin(), msg.position.end(), 0.0);
  std::fill(msg.velocity.begin(), msg.velocity.end(), 0.0);
  std::fill(msg.effort.begin(), msg.effort.end(), 0.0);
  ros::Time t0 = ros::Time::now();
  msg.header.stamp = t0;
  
  ros::Rate lp(1.0/srv.request.delta_time);
  ros::Publisher target_js_pub = nh.advertise<sensor_msgs::JointState>("joint_target", 1000);
  for (int idx=0;idx<2;idx++)
  {
    target_js_pub.publish(msg);
    ros::spinOnce();
    lp.sleep();
  }
  
  
  for (int iPnt = 0;iPnt<trajectory_final.points.size();iPnt++)
  {
    msg.position = trajectory_final.points.at(iPnt).positions;
    msg.velocity = trajectory_final.points.at(iPnt).velocities;
    msg.header.stamp = t0 + trajectory_final.points.at(iPnt).time_from_start;
    target_js_pub.publish(msg);
    ros::spinOnce();
    lp.sleep();
    if (!ros::ok())
      return 0;
  }
  
}