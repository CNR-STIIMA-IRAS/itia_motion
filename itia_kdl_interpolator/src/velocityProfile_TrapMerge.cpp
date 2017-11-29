
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

#include "itia_kdl_interpolator/velocityProfile_TrapMerge.hpp"
 
namespace KDL {

	VelocityProfile_TrapMerge::VelocityProfile_TrapMerge(double _maxvel,double _maxacc,bool _is_breaking):
												maxvel(_maxvel),maxacc(_maxacc),is_breaking(_is_breaking) {}
 	
	void VelocityProfile_TrapMerge::SetMax(double _maxvel,double _maxacc,bool _is_breaking) {
		maxvel = _maxvel; maxacc = _maxacc; is_breaking = _is_breaking;
	}
 
	void VelocityProfile_TrapMerge::SetProfile(double pos1,double pos2, double vel1, double vel2, bool& _is_complete) {
		startpos        = pos1;
		endpos          = pos2;
		startvel		= vel1;
		endvel			= vel2;
		
		if(is_breaking){  //se is_breaking è true (fase decelerazione) viene presa in considerazione la endvel e ricalcolata la startvel (il valore in ingresso non viene considerato)
			
			double vel = sqrt(pow(endvel,2.0)+2*maxacc*fabs(endpos-startpos));
			double a = -maxacc * sign(endpos-startpos);		
			if (vel > maxvel){
				t3 = (maxvel-fabs(endvel))/maxacc;
				startvel = maxvel * sign(endpos-startpos);
				double pos_t2 = endpos-((startvel-endvel)*t3)/2.0;
				t2 = fabs(pos_t2-startpos)/maxvel;
				_is_complete = true;        // indica se nel tratto in esame è stato possibile accelerare fino a velmax o frenare partendo da velmax
			} else {
				t3 = (vel-fabs(endvel))/maxacc;
				startvel = vel * sign(endpos-startpos);
				t2 = 0;
				_is_complete = false;
			}
			t1 = 0;
			
		} else {  //se is_breaking è false (fase accelerazione) viene presa in considerazione la startvel e ricalcolata la endvel (il valore in ingresso non viene considerato)
			
			double vel = sqrt(pow(startvel,2.0)+2*maxacc*fabs(endpos-startpos));
			double a = maxacc * sign(endpos-startpos);
			if(vel > maxvel){
				t1 = (maxvel-fabs(startvel))/maxacc;
				endvel = maxvel * sign(endpos-startpos);
				double pos_t1 = startpos + startvel*t1 + a/2*pow(t1,2.0);
				t2 = fabs(endpos-pos_t1)/maxvel;
				_is_complete = true;
			} else {
				t1 = (vel-fabs(startvel))/maxacc;
				endvel = vel * sign(endpos-startpos);
				t2 = 0;
				_is_complete = false;
			}
			t3 = 0;
			
		}
		duration = t1 + t2 + t3;
		std::cout << "is_breaking: " << is_breaking << " , startvel: " << startvel << " , endvel: " << endvel << std::endl;
		std::cout << "durata: " << duration << " = " << t1 << " + " << t2 << " + " << t3 << std::endl;
		
		a1 = startpos;
		a2 = startvel;
		a3 = maxacc * sign(endpos-startpos) / 2.0;
		
		b1 = a1 + a2*t1 + a3*t1*t1;
		b2 = maxvel;
		b3 = 0;
		
		c1 = b1 + b2*t2;
		c2 = maxvel;
		c3 = -maxacc * sign(endpos-startpos) / 2.0;
		
		vel1 = startvel;
		vel2 = endvel;
		
		return;
		
	}

 	void VelocityProfile_TrapMerge::SetProfileDuration(double pos1,double pos2, double vel1, double vel2,double newduration) {
		// duration should be longer than originally planned duration
		// Fastest :
		bool flag;
		SetProfile(pos1,pos2,vel1,vel2,flag);
		// Must be Slower  :
		double factor = duration/newduration;
		if (factor > 1)
			return; // do not exceed max

		a2*=factor;
		a3*=factor*factor;
		b2*=factor;
		b3*=factor*factor;
		c2*=factor;
		c3*=factor*factor;
		duration = newduration;
		t1 /= factor;
		t2 /= factor;
		t3 /= factor;
	}
  
	double VelocityProfile_TrapMerge::Duration() const {
		return duration;
	}
 
	double VelocityProfile_TrapMerge::Pos(double time) const {
		if (time < 0) {
			return startpos;
		} else if (time<t1) {
			return a1+time*(a2+a3*time);
		} else if (time<(t1+t2)) {
			return b1+(time-t1)*(b2+b3*(time-t1));
		} else if (time<duration) {
			return c1+(time-t1-t2)*(c2+c3*(time-t1-t2));
		} else {
			return endpos;
		}
	}
	
	double VelocityProfile_TrapMerge::Vel(double time) const {
		if (time < 0) {
			return 0;
		} else if (time<t1) {
			return a2+2*a3*time;
		} else if (time<(t1+t2)) {
			return b2+2*b3*(time-t1);
		} else if (time<duration) {
			return c2+2*c3*(time-t1-t2);
		} else {
			return 0;
		}
	}
 
	double VelocityProfile_TrapMerge::Acc(double time) const {
		if (time < 0) {
			return 0;
		} else if (time<t1) {
			return 2*a3;
		} else if (time<(t1+t2)) {
			return 2*b3;
		} else if (time<duration) {
			return 2*c3;
		} else {
			return 0;
		}
	}
 
	VelocityProfile* VelocityProfile_TrapMerge::Clone() const {
		VelocityProfile_TrapMerge* res =  new VelocityProfile_TrapMerge(maxvel,maxacc, is_breaking);
		res->SetProfileDuration( this->startpos, this->endpos, this->startvel, this->endvel, this->duration );
		return res;
	}
 
	VelocityProfile_TrapMerge::~VelocityProfile_TrapMerge() {}

	void VelocityProfile_TrapMerge::Write(std::ostream& os) const {
		os << "TRAPEZOIDALMERGE[" << maxvel << "," << maxacc << "," << is_breaking << "]";
	}
 
}
