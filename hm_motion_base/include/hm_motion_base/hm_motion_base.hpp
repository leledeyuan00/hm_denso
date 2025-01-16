#ifndef HM_MOTION_BASE_HPP__
#define HM_MOTION_BASE_HPP__

// std
#include <stdlib.h>
#include <sstream>
#include <ctime>
#include <thread>

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
#include "hm_motion_base/hm_tasks.hpp"

// eigen
#include <eigen3/Eigen/Dense>

namespace hm_research
{


class HmMotionBase : public rclcpp::Node
{

public:
    HmMotionBase(const std::string& node_name):Node(node_name){
        init();
    };
    void start();

    void activate(){
        robot_.target_pose = robot_.target_monitor;
        robot_.start_pose  = robot_.target_monitor;
        active_ = true;
    };
    void deactivate(){active_ = false;};

protected:

    /* functions */
    void init();
    virtual void custom_init(){};

    virtual geometry_msgs::msg::PoseStamped intepolation(geometry_msgs::msg::PoseStamped init_pose, geometry_msgs::msg::PoseStamped target_pose,
                        double duration, double current_duration)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = init_pose.header.frame_id;
        pose.pose.position.x = init_pose.pose.position.x + (target_pose.pose.position.x - init_pose.pose.position.x) * current_duration / duration;
        pose.pose.position.y = init_pose.pose.position.y + (target_pose.pose.position.y - init_pose.pose.position.y) * current_duration / duration;
        pose.pose.position.z = init_pose.pose.position.z + (target_pose.pose.position.z - init_pose.pose.position.z) * current_duration / duration;

        Eigen::Quaterniond init_q, target_q;
        tf2::fromMsg(init_pose.pose.orientation, init_q);
        tf2::fromMsg(target_pose.pose.orientation, target_q);

        Eigen::Quaterniond q = init_q.slerp(current_duration / duration, target_q);
        pose.pose.orientation = tf2::toMsg(q);

        return pose;
    }

    virtual geometry_msgs::msg::WrenchStamped intepolation_wrench(geometry_msgs::msg::WrenchStamped init_wrench, geometry_msgs::msg::WrenchStamped target_wrench,
                            double duration, double current_duration)
    {
        geometry_msgs::msg::WrenchStamped wrench;
        wrench.header = init_wrench.header;
        wrench.wrench.force.x = init_wrench.wrench.force.x + (target_wrench.wrench.force.x - init_wrench.wrench.force.x) * current_duration / duration;
        wrench.wrench.force.y = init_wrench.wrench.force.y + (target_wrench.wrench.force.y - init_wrench.wrench.force.y) * current_duration / duration;
        wrench.wrench.force.z = init_wrench.wrench.force.z + (target_wrench.wrench.force.z - init_wrench.wrench.force.z) * current_duration / duration;
        wrench.wrench.torque.x = init_wrench.wrench.torque.x + (target_wrench.wrench.torque.x - init_wrench.wrench.torque.x) * current_duration / duration;
        wrench.wrench.torque.y = init_wrench.wrench.torque.y + (target_wrench.wrench.torque.y - init_wrench.wrench.torque.y) * current_duration / duration;
        wrench.wrench.torque.z = init_wrench.wrench.torque.z + (target_wrench.wrench.torque.z - init_wrench.wrench.torque.z) * current_duration / duration;

        return wrench;
    }


    // robot state
    HmRobotState get_robot_state() {return robot_;};
    SystemState get_system_state() {return system_state_;};


    // set robot state
    void set_target_pose(geometry_msgs::msg::PoseStamped target_pose) {robot_.target_pose = target_pose;};
    void set_robotiq(std_msgs::msg::Float64MultiArray robotiq) {robot_.robotiq = robotiq;};

    /* FSM tasks */ 
    /* @brief from start_pose(initialized when this task running) to target_pose
    *  @param target pose
    *  @param duration 
    *  @return finish or not
    */ 
    bool move(geometry_msgs::msg::PoseStamped pose, double duration);

    /* @brief from start_pose(initialized when this task running) to target_pose
    *  @param target pose
    *  @param duration 
    *  @return finish or not
    */
    bool move_wrench(geometry_msgs::msg::PoseStamped pose, 
                            geometry_msgs::msg::WrenchStamped wrench, double duration);

    /*
    * @brief sleep for a while
    * @param duration
    * @return finish or not
    */
    bool sleep(double duration);

    /* @brief joint move
    *  @param target
    *  @return finish or not
    */
    bool joint_move(std::vector<double> joints, double time);

    
    /*
    * @brief Push back the task, the task is derived from HmTaskBase
    * @param task 
    */
    uint8_t task_pushback(std::shared_ptr<hm_research::HmTask> task);
    virtual void tasks_init() = 0;
    void task_execute();

    /// @brief set the task finished
    void set_task_finished(){
        next_task_offset_ = 1;
        tasks_vector_[system_state_.task_num]->set_state(HmTaskState::FINISH);
    };
    void goto_specific_task(uint8_t task_num){
        next_task_offset_ = task_num - system_state_.task_num;
        tasks_vector_[system_state_.task_num]->set_state(HmTaskState::FINISH);
    };
    void goto_init_task(){
        goto_specific_task(0);
    };
    void goto_last_task(){
        goto_specific_task(tasks_vector_.size()-1);
    };

    // states
    std::shared_ptr<HmFlags> system_flags_;
    bool initialized_ = false;
    inline bool check_init() { return initialized_; }

    SystemState system_state_; // current task number, current time, start time
    uint8_t next_task_offset_ = 1;
    
    // std
    std::thread control_loop_thread_;

    // time
    rclcpp::Clock my_clock_;

    // system state
    // create a vector for TaskWrapper
    std::vector<std::shared_ptr<hm_research::HmTask>> tasks_vector_;
    
    
    bool active_ = true;


private:

    /* function */
    void control_loop();
    // ros
    void ros_init();
    

    /* variable */ 
    // ros
    // pub
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr robotiq_pub_;

    // sub
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_monitor_sub_;

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

    // srvs
    rclcpp::Client<cartesian_controller_msgs::srv::JointMove>::SharedPtr joint_move_client_;

    // robot state
    HmRobotState robot_;

};

} // namespace name




#endif // HM_MOTION_BASE_HPP__