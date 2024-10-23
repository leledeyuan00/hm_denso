#ifndef HM_MOTION_UTILS_HPP__
#define HM_MOTION_UTILS_HPP__

#include "hm_motion_base/hm_motion_base.hpp"

namespace hm_research
{

void set_active(HmMotionBase &hm_motion_base)
{
    hm_motion_base.activate();
}

void set_inactive(HmMotionBase &hm_motion_base)
{
    hm_motion_base.deactivate();
}


} // namespace hm_research



#endif // HM_MOTION_UTILS_HPP__