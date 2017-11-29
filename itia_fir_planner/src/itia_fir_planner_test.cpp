#include <itia_fir_planner/itia_fir_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <itia_rutils/itia_rutils.h>
#include <itia_fir_planner/velocity_profile.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fir_planner_test");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::Publisher target_js_pub = nh.advertise<sensor_msgs::JointState>("joint_target", 1);

  std::string  parameter_name = "fir";
  
  
  int dimension = 1;
  if (!nh.getParam(parameter_name+"/dimension", dimension))
  {
    ROS_FATAL("%s", (parameter_name+"/dimension does not exist").c_str());
    throw std::invalid_argument(parameter_name+"/dimension does not exist");
  }
  
  double st = 1e-1;
  if (!nh.getParam(parameter_name+"/sample_period", st))
    throw std::invalid_argument(parameter_name+"/sample_period does not exist");
  
  ros::Rate loop_rate(1/st);
  
  itia::JMotion motion(3, dimension);
  Eigen::VectorXd qini(dimension);
  sensor_msgs::JointState msg;
  msg.position.resize(dimension);
  msg.velocity.resize(dimension);
  msg.effort.resize(dimension);
  std::fill(msg.position.begin(), msg.position.end(), 0.0);
  std::fill(msg.velocity.begin(), msg.velocity.end(), 0.0);
  std::fill(msg.effort.begin(), msg.effort.end(), 0.0);
  msg.name.resize(dimension);
  for (int idx = 0;idx<dimension;idx++)
    msg.name.at(idx) = "joint_"+std::to_string(idx+1);
    
  ros::Time t0 = ros::Time::now();
  while ( (ros::Time::now()-t0).toSec() <1);
  {
    msg.header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    loop_rate.sleep();
  }
    
  qini.setZero();
  
  int npnt = 3;
  Eigen::MatrixXd waypoints(npnt, dimension);
  Eigen::MatrixXd waypoints_vel(npnt, dimension);
  Eigen::VectorXd waypoints_accuracy(npnt, 1);
  Eigen::MatrixXd aux;
  Eigen::VectorXd time(npnt, 1);
  
  time<<       0, 0, 0;                               //, 1.03, 1.04, 4.05, 4.06;
  waypoints << 0, 1, 0;                                  //,    0,    0,    2,    2;
  waypoints_vel.setConstant(1);
  waypoints_accuracy.setConstant(0.001);
  
  unsigned int np = 2;
  unsigned int nfir = 100;
  itia::helios::VelocityProfile velocity_profile(qini, np, nfir, st);
  
  velocity_profile.addPoints(time, waypoints.transpose(), waypoints_vel.transpose(), waypoints_accuracy);
  
  t0 = ros::Time::now();
  while (ros::ok())
  {
    double time;
    Eigen::VectorXd qact;
    Eigen::VectorXd Dqact;
    Eigen::VectorXd DDqact;
    if ( ( (ros::Time::now()-t0).toSec()>0.8 && (ros::Time::now()-t0).toSec() <5 ) || ( (ros::Time::now()-t0).toSec()>5.5 && (ros::Time::now()-t0).toSec() < 7 ) )
      velocity_profile.softStop(time, qact, Dqact, DDqact);
    else
    {
      ros::Time t1 = ros::Time::now();
      velocity_profile.update(time, qact, Dqact, DDqact);
      ROS_INFO("dt=%f [ms]", (ros::Time::now()-t1).toSec() *1000);
    }
    for (int iDim = 0;iDim<dimension;iDim++)
    {
      msg.position.at(iDim) = qact(iDim);
      msg.velocity.at(iDim) = Dqact(iDim);
      msg.effort.at(iDim) = DDqact(iDim);
    }
    msg.header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    
    loop_rate.sleep();
  }
  return 0;
  itia::helios::FirPlanner planner(nh, parameter_name, qini);
  
  t0 = ros::Time::now();
  planner.setNextPoints(time, waypoints);
  while (ros::ok())
  {
    motion = planner.update();
    for (int iDim = 0;iDim<dimension;iDim++)
    {
      msg.position.at(iDim) = motion(0, iDim);
      msg.velocity.at(iDim) = motion(1, iDim);
      msg.effort.at(iDim)   = motion(2, iDim);
    }
    msg.header.stamp = ros::Time::now();
    target_js_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;  
}