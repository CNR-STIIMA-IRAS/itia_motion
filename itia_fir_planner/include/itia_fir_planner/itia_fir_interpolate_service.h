#ifndef __ITIA_FIR_INTERPOLATE_SERVICE__
# include <itia_motion_msgs/InterpolatePnt.h>
#include <itia_fir_planner/velocity_profile.h>

namespace itia{
namespace helios{

bool interpolateTrajectories(itia_motion_msgs::InterpolatePntRequest& req, itia_motion_msgs::InterpolatePntResponse& res);

}
}

#endif