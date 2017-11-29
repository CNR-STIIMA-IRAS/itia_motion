#ifndef KDL_MOTION_VELOCITYPROFILE_TRAPMERGE_H
#define KDL_MOTION_VELOCITYPROFILE_TRAPMERGE_H
 
#include <kdl/velocityprofile.hpp>
 

namespace KDL {

	class VelocityProfile_TrapMerge : public VelocityProfile {
		
			// For "running" a motion profile :
			double a1,a2,a3; // coef. from ^0 -> ^2 of first part
			double b1,b2,b3; // of 2nd part
			double c1,c2,c3; // of 3th part
			double duration;
			double t1,t2,t3;

			double startpos;
			double endpos;
			double startvel;
			double endvel;
	
			// Persistent state :
			double maxvel;
			double maxacc;
			bool   is_breaking;
			

		public:

			VelocityProfile_TrapMerge(double _maxvel=0,double _maxacc=0,bool _is_breaking=false);

			void SetMax(double _maxvel,double _maxacc);
            void SetMax(double _maxvel,double _maxacc,bool _is_breaking);
 
            virtual void SetProfile(double pos1,double pos2){};
			virtual void SetProfile(double pos1, double pos2, double vel1, double vel2, bool& _is_complete);
			 
			virtual void SetProfileDuration(double pos1,double pos2,double newduration){};
            virtual void SetProfileDuration(double pos1,double pos2,double vel1,double vel2,double newduration);

            virtual double Duration() const;
            virtual double Pos(double time) const;
            virtual double Vel(double time) const;
            virtual double Acc(double time) const;
            virtual void Write(std::ostream& os) const;
            virtual VelocityProfile* Clone() const;
 
            virtual ~VelocityProfile_TrapMerge();
    };

}
 
#endif