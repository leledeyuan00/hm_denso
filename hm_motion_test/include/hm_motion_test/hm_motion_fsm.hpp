#ifndef HM_GRASP_TASKS_HPP__
#define HM_GRASP_TASKS_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <ctime>
#include <fstream>
#include <thread>

// ros
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "kdl/frames.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

// custom
#include "cartesian_controller_msgs/srv/joint_move.hpp"

#include "hm_motion_base/hm_motion_base.hpp"

namespace hm_research
{

class HmMotionFsm : public HmMotionBase
{
public:
    HmMotionFsm();

private:
    /* function */
    virtual void custom_init();
    // tasks
    void test_service(bool on);
    virtual void tasks_init();


    /* variable */
    // srvs
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr test_client_;

    // calibration
    Eigen::Isometry3d calibration_matrix_r_;

};

} // namespace hm_research


#endif // HM_SERVO_TRACKING_HPP__