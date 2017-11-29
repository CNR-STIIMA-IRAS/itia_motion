#include <itia_fir_planner/itia_fir_interpolate_service.h>
namespace itia 
{
namespace helios 
{

bool interpolateTrajectories( itia_motion_msgs::InterpolatePntRequest&  req, 
                              itia_motion_msgs::InterpolatePntResponse& res )
{
  
  res.trajectory_full.joint_names = req.trajectory_waypoints.joint_names;
  int dimension = req.trajectory_waypoints.points.at(0).positions.size(); 
   
  Eigen::MatrixXd motion(3, dimension);
  Eigen::VectorXd qini(dimension);
  
  int nPnt = req.trajectory_waypoints.points.size();
  Eigen::MatrixXd waypoints(nPnt, dimension);
  Eigen::MatrixXd waypoints_vel(nPnt, dimension);
  Eigen::VectorXd waypoints_accuracy(nPnt, 1);
  Eigen::VectorXd time(nPnt, 1);
  
  double path_accuracy;
  if (req.path_tolerance.size()>0)
    path_accuracy = req.path_tolerance.at(0).position;
  else
    if (!ros::param::get("fir/default_path_accuracy", path_accuracy))
    {
      ROS_WARN("PARAMETER 'default_path_accuracy' DOES NOT EXIST,  set default value equal to 0.0");
      path_accuracy = 0.0;
    }
  waypoints_accuracy.setConstant(path_accuracy);
  
  double goal_accuracy;
  if (req.goal_tolerance.size()>0)
    goal_accuracy = req.goal_tolerance.at(0).position;
  else
    if (!ros::param::get("fir/default_goal_accuracy", goal_accuracy))
    {
      ROS_WARN("PARAMETER 'fir/default_goal_accuracy' DOES NOT EXIST,  set default value equal to 0.0");
      goal_accuracy = 0.0;
    }
  waypoints_accuracy(waypoints_accuracy.rows()-1) = goal_accuracy;
  
  
  double st = req.delta_time;

  for (int iPnt = 0;iPnt<nPnt;iPnt++)
  {
    for (int iDim = 0;iDim<dimension;iDim++)
    {
      waypoints(iPnt, iDim) = req.trajectory_waypoints.points.at(iPnt).positions.at(iDim);
      waypoints_vel(iPnt, iDim) = req.max_speed;
    }
    time(iPnt) = req.trajectory_waypoints.points.at(iPnt).time_from_start.toSec();
  } 
  qini = waypoints.row(0).transpose();
  
  int np = 2;
  int nfir = 100;
  double acc_vel_ratio=10;
  if (!ros::param::get("fir/acc_vel_ratio", nfir))
  {
    ROS_WARN("PARAMETER 'fir/acc_vel_ratio' DOES NOT EXIST,  set default value equal to 10");
    acc_vel_ratio=10;
  }
  nfir = int(std::ceil(std::abs(1.0/st/acc_vel_ratio)));

  itia::helios::VelocityProfile velocity_profile(qini, np, nfir, st);
  
  velocity_profile.addPoints(time, waypoints.transpose(), waypoints_vel.transpose(), waypoints_accuracy);
  
  trajectory_msgs::JointTrajectoryPoint pnt;
  pnt.positions.resize(dimension);
  pnt.velocities.resize(dimension);
  pnt.accelerations.resize(dimension);
  pnt.effort.resize(dimension);
  std::fill(pnt.effort.begin(), pnt.effort.end(), 0.0);
  
  
  bool is_finished = false;
  
  while (ros::ok() && !is_finished && res.trajectory_full.points.size()<1e7)
  {
    double time;
    Eigen::VectorXd qact;
    Eigen::VectorXd Dqact;
    Eigen::VectorXd DDqact;
    
    is_finished = velocity_profile.update(time, qact, Dqact, DDqact);
    
    for (int iDim = 0;iDim<dimension;iDim++)
    {
      pnt.positions.at(iDim) = qact(iDim);
      pnt.velocities.at(iDim) = Dqact(iDim);
      pnt.accelerations.at(iDim) = DDqact(iDim);
    }
    
    pnt.time_from_start = ros::Duration(time);
    res.trajectory_full.points.push_back(pnt);
    ROS_INFO_THROTTLE(5, "res.trajectory_full.points.size() = %zu", res.trajectory_full.points.size());
  }
  
  if (!is_finished)
    ROS_WARN("Exceeded from the maximum number of points. Points interpolated: %zu ", res.trajectory_full.points.size() );
      
  return is_finished;
}


}
}