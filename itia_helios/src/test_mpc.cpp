
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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <itia_rutils/itia_rutils.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "helios_mpc_test");
  ros::NodeHandle nh;
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::Publisher target_js_pub = nh.advertise<sensor_msgs::JointState>("joint_target", 1);

  std::string  parameter_name = "helios";
  
  int dimension;
  if (!nh.getParam(parameter_name+"/dimension", dimension))
    throw std::invalid_argument(parameter_name+"/dimension does not exist");
  
  double st;
  if (!nh.getParam(parameter_name+"/sample_period", st))
    throw std::invalid_argument(parameter_name+"/sample_period does not exist");
  ros::Rate loop_rate(1/st);
  
  itia::JMotion motion(3, dimension);
  Eigen::VectorXd qini(dimension);
  sensor_msgs::JointState msg;
  msg.position.resize(dimension);
  msg.velocity.resize(dimension);
  msg.effort.resize(dimension);
  msg.name.resize(dimension);
  msg.name.at(0) = "joint_1";
  msg.name.at(1) = "joint_2";
  msg.name.at(2) = "joint_3";
  msg.name.at(3) = "joint_4";
  msg.name.at(4) = "joint_5";
  msg.name.at(5) = "joint_6";
  
  qini.setZero();
  
  Eigen::MatrixXd qfin;
  Eigen::MatrixXd aux;
  Eigen::VectorXd time;
  
  if (!itia::rutils::getParam(nh, "q", qfin))
    return -1;
  
  int npnt = qfin.rows();
  if (!itia::rutils::getParam(nh, "time", aux))
    return -1;
  time = aux;

  itia::helios::MpcPlanner planner(nh, parameter_name, qini);
      
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