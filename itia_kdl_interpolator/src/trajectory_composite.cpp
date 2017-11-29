
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

// Modifica del metodo Destroy() con aggiunta azzeramento duration

#include <kdl/trajectory_composite.hpp>
#include <kdl/path_composite.hpp>
 
namespace KDL {
 
    using namespace std;
 
 
    Trajectory_Composite::Trajectory_Composite():duration(0.0)
    {
    }
 
	double Trajectory_Composite::Duration() const{
		return duration;
	}
 
	Frame Trajectory_Composite::Pos(double time) const {
		// not optimal, could be done in log(#elem)
		// or one could buffer the last segment and start looking from there.
		unsigned int i;
		double previoustime;
		Trajectory* traj;
		if (time < 0) {
			return vt[0]->Pos(0);
		}
		previoustime = 0;
		for (i=0;i<vt.size();i++) {
			if (time < vd[i]) {
				return vt[i]->Pos(time-previoustime);
			}
			previoustime = vd[i];
		}
		traj = vt[vt.size()-1];
		return traj->Pos(traj->Duration());
	}
 
	Twist Trajectory_Composite::Vel(double time) const {
		// not optimal, could be done in log(#elem)
		unsigned int i;
		Trajectory* traj;
		double previoustime;
		if (time < 0) {
			return vt[0]->Vel(0);
		}
		previoustime = 0;
		for (i=0;i<vt.size();i++) {
			if (time < vd[i]) {
				return vt[i]->Vel(time-previoustime);
			}
			previoustime = vd[i];
		}
		traj = vt[vt.size()-1];
		return traj->Vel(traj->Duration());
    }
 
	Twist Trajectory_Composite::Acc(double time) const {
		// not optimal, could be done in log(#elem)
		unsigned int i;
		Trajectory* traj;
		double previoustime;
		if (time < 0) {
			return vt[0]->Acc(0);
		}
		previoustime = 0;
		for (i=0;i<vt.size();i++) {
			if (time < vd[i]) {
				return vt[i]->Acc(time-previoustime);
			}
			previoustime = vd[i];
		}
		traj = vt[vt.size()-1];
		return traj->Acc(traj->Duration());
    }
 
	void Trajectory_Composite::Add(Trajectory* elem) {
		vt.insert(vt.end(),elem);
		duration += elem->Duration();
		vd.insert(vd.end(),duration);
    }
 
	void Trajectory_Composite::Destroy() {
		VectorTraj::iterator it;
		for (it=vt.begin();it!=vt.end();it++) {
			delete *it;
		}
		vt.erase(vt.begin(),vt.end());
		vd.erase(vd.begin(),vd.end());
		duration = 0.0;
    }
 
	Trajectory_Composite::~Trajectory_Composite() {
		Destroy();
    }
 
	void Trajectory_Composite::Write(ostream& os) const {
		os << "COMPOSITE[ " << vt.size() << endl;
		unsigned int i;
		for (i=0;i<vt.size();i++) {
			vt[i]->Write(os);
		}
		os << "]" << endl;
    }
 
	Trajectory* Trajectory_Composite::Clone() const{
		Trajectory_Composite* comp = new Trajectory_Composite();
		for (unsigned int i = 0; i < vt.size(); ++i) {
			comp->Add(vt[i]->Clone());
		}
		return comp;
	}
}