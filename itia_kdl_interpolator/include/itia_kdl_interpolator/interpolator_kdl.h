#ifndef ____INTERPOLATOR_KDL____
#define ____INTERPOLATOR_KDL____


#include <ros/ros.h>
#include <ros/package.h>
#include <itia_futils/itia_futils.h>
#include <itia_tutils/itia_tutils.h>
#include <itia_gutils/itia_gutils.h>
#include <itia_comau_msgs/CncPos.h>
#include <itia_comau_msgs/JointPos.h>
#include <itia_comau_msgs/CartPnt.h>
#include <itia_comau_msgs/trj.h>
#include <itia_motion_msgs/CartPnt_at_Time.h>
#include <itia_motion_msgs/reloadReferences.h>

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_line.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_composite.hpp>

#include <exception>
#include <iostream>
#include "itia_kdl_interpolator/velocityProfile_TrapMerge.hpp"


namespace unibs{
  namespace deburring{
    
		class InterpolatorKDL
		{
			private:
				ros::NodeHandle* m_ptr_nh;
				ros::Rate m_loop_rate;
				
				const double PI = 3.141592653589793;
				
				Eigen::Array<double,6,1> m_bframe, m_tframe, m_uframe, m_endstroke_pos, m_endstroke_neg, m_q_avg;
				int m_err_counter = 0;
				int m_arm_num = 2;
				KDL::Chain m_urdf_chain;
				KDL::Trajectory_Composite m_full_trj;
				KDL::Frame m_actual_pos;   // posizione attuale del robot da tool a base
				double m_actual_time;      // tempo dell'ultima chiamata alla funzione getPose
				double m_time_offset;      // tiene traccia dei tempi quando la traiettoria viene ricaricata durante l'esecuzione
				bool m_initial_pos_set = false;
        
        void currentPos_cb(const itia_comau_msgs::CartPnt::ConstPtr& pos);
				
			public:
				InterpolatorKDL(ros::NodeHandle* ptr_nh);
				~InterpolatorKDL(){};
				bool loadTrj( itia_comau_msgs::trj::Request& req, itia_comau_msgs::trj::Response& res);
				bool appendTrj( itia_comau_msgs::trj::Request& req, itia_comau_msgs::trj::Response& res);
				bool getPose( itia_motion_msgs::CartPnt_at_Time::Request& req, itia_motion_msgs::CartPnt_at_Time::Response& res );
				bool reloadReferences(itia_motion_msgs::reloadReferences::Request& req, itia_motion_msgs::reloadReferences::Response& res);
				
		};
	}
}
	
#endif
