
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
