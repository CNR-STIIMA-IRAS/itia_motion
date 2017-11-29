#include "itia_kdl_interpolator/interpolator_kdl.h"

int main ( int argc, char** argv ) 
{
  printf ( " [ %s%s:%d%s ]\t *********************************************\n",GREEN, __FUNCFILE__, __LINE__, RESET );
  printf ( " [ %s%s:%d%s ]\t ******** INTERPOLATOR SERVER KDL 1.0 ********\n",GREEN, __FUNCFILE__, __LINE__, RESET );
  printf ( " [ %s%s:%d%s ]\t *********************************************\n",GREEN, __FUNCFILE__, __LINE__, RESET );

  printf ( " [ %s%s:%d%s ]\t Init ROS\n",GREEN, __FUNCFILE__, __LINE__, RESET );
  // ------ Init ROS ------
  ros::init ( argc, argv,"interpolator_KDL_server" );
  ros::NodeHandle nh;

	unibs::deburring::InterpolatorKDL server(&nh);
		
	ros::ServiceServer  loadTrj_srv=nh.advertiseService("loadTrj", &unibs::deburring::InterpolatorKDL::loadTrj, &server);
	ros::ServiceServer  appendTrj_srv=nh.advertiseService("appendTrj", &unibs::deburring::InterpolatorKDL::appendTrj, &server);
	ros::ServiceServer  getPose_srv=nh.advertiseService("getPose", &unibs::deburring::InterpolatorKDL::getPose, &server);
	ros::ServiceServer  reloadRefs_srv=nh.advertiseService("reloadRefs", &unibs::deburring::InterpolatorKDL::reloadReferences, &server);
	
	ros::spin();
  return 0;
}
