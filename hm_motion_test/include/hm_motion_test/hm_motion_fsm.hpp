#ifndef HM_MOTION_TEST_HPP__
#define HM_MOTION_TEST_HPP__

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

class HmMotionFsm : public rclcpp::Node
{
public:
    HmMotionFsm();
    void start();
    void control_loop();

private:
    /* function */
    // ros
    void ros_init();
    geometry_msgs::msg::PoseStamped intepolation(geometry_msgs::msg::PoseStamped &init_pose, geometry_msgs::msg::PoseStamped target_pose,
                        double current_duration, double duration);
    geometry_msgs::msg::WrenchStamped intepolation_wrench(geometry_msgs::msg::WrenchStamped &init_wrench, geometry_msgs::msg::WrenchStamped target_wrench,
                        double current_duration, double duration);
    
    // FSM tasks
    void tasks_init();
    void tasks_execute();
    void task_sleep(rclcpp::Time start_time, double duration);
    void task_joint_move(std::vector<double> joints, double time);
    void task_intepolation(geometry_msgs::msg::PoseStamped pose, double duration);
    void task_intepolation_with_wrench(geometry_msgs::msg::PoseStamped pose, geometry_msgs::msg::WrenchStamped wrench,  double duration);
    void test_service(bool on);
    /* variable */
    // ros
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_ee_tf_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_target_monitor_sub_;    
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

    // srvs
    rclcpp::Client<cartesian_controller_msgs::srv::JointMove>::SharedPtr joint_move_client_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr test_client_;
    // states
    bool initialized_ = false;
    bool service_called_ = false;
    bool new_task_ = false;
    bool model_joint_ = false;

    // std
    std::thread control_loop_thread_;

    // time
    rclcpp::Clock my_clock_;
    rclcpp::Time start_time_;

    // system state
    uint8_t system_state_ = 0;


    // robot state
    geometry_msgs::msg::PoseStamped current_ee_tf_;
    geometry_msgs::msg::PoseStamped home_ee_tf_;

    geometry_msgs::msg::PoseStamped target_pose_;
    
    geometry_msgs::msg::PoseStamped target_temp_pose_; // for each start of the task
    geometry_msgs::msg::PoseStamped target_monitor_; 

    geometry_msgs::msg::WrenchStamped current_wrench_;
    geometry_msgs::msg::WrenchStamped target_wrench_;
    geometry_msgs::msg::WrenchStamped target_temp_wrench_; // for each start of the task

    // calibration
    Eigen::Isometry3d calibration_matrix_;

    std::vector<std::function<void()>> tasks_vector_;

};




#endif // hm_denso_SERVO_TRACKING_HPP__