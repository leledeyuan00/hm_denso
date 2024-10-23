#ifndef HM_TASKS_HPP__
#define HM_TASKS_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <ctime>
#include <thread>
#include <cstring>
// ros
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

#include "kdl/frames.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

// custom
#include "cartesian_controller_msgs/srv/joint_move.hpp"

// eigen
#include <eigen3/Eigen/Dense>

#define TaskPtr std::make_shared<hm_research::HmTask>

namespace hm_research
{

// enum class for task state
enum class HmTaskState
{
    INIT = 0,
    EXECUTE,
    FINISH
};

// struct for Hm flags
struct HmFlags
{
    bool service_called = false;
    bool model_joint = false;
};

struct HmRobotState
{
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::PoseStamped target_monitor; // the current target pose from the controller for changing control from different process program
    geometry_msgs::msg::PoseStamped target_pose;
    geometry_msgs::msg::WrenchStamped current_wrench;
    geometry_msgs::msg::WrenchStamped target_wrench;
    std_msgs::msg::Float64MultiArray robotiq;
    geometry_msgs::msg::PoseStamped start_pose;
};

struct SystemState
{
    uint8_t task_num = 0;
    rclcpp::Time start_time;
    rclcpp::Time current_time;
};


// A wrapper class for Hm task to control function sequence
class HmTask
{

public:

    /// @brief Initialize the task with task_name and execute function
    /// @param task_name 
    /// @param execute 
    HmTask(std::string task_name, std::function<void()> execute)
        : task_name_(task_name), execute_(execute){
            init_ = [](){}; // empty init function
        };
    
    /// @brief Initialize the task with task_name, init function, and execute function
    /// @param task_name 
    /// @param init 
    /// @param execute 
    HmTask(std::string task_name, std::function<void()> init, std::function<void()> execute)
        : task_name_(task_name), init_(init), execute_(execute){};

    void init(){
        init_();
    }

    void execute(){
        execute_();
    }

    std::string get_name(){
        return task_name_;
    }

    HmTaskState get_state(){
        return state_;
    }

    void set_state(HmTaskState state){
        state_ = state;
    }

private:
    std::string task_name_;
    std::function<void()> init_;
    std::function<void()> execute_;

    HmTaskState state_ = HmTaskState::INIT;
};


} // namespace hm_research

#endif // HM_TASKS_HPP__