#include "hm_motion_base/hm_motion_base.hpp"

namespace hm_research
{

void HmMotionBase::init()
{
    ros_init();
}

void HmMotionBase::ros_init()
{
    // clock
    my_clock_ = rclcpp::Clock(RCL_ROS_TIME);

    // pub
    target_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/cartesian_compliance_controller/target_frame", rclcpp::SystemDefaultsQoS());
    wrench_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/cartesian_compliance_controller/target_wrench", rclcpp::SystemDefaultsQoS());
    robotiq_pub_ = 
        this->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_forward_position_controller/commands", rclcpp::SystemDefaultsQoS());
    
    // sub
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cartesian_compliance_controller/current_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            robot_.current_pose = *msg;
            if (!initialized_)
            {
                robot_.target_pose = *msg;
                robot_.start_pose = *msg;
                system_state_.start_time = my_clock_.now();
                initialized_ = true;
            }
        });

    target_monitor_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cartesian_compliance_controller/target_frame_monitor", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            robot_.target_monitor = *msg;
        });


    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/cartesian_compliance_controller/current_wrench", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            robot_.current_wrench = *msg;
        });

    // srvs
    joint_move_client_ = this->create_client<cartesian_controller_msgs::srv::JointMove>("/cartesian_compliance_controller/target_joint");

    system_flags_.reset(new HmFlags);
}

/* Predefined Functions */

bool HmMotionBase::move(geometry_msgs::msg::PoseStamped pose, double duration)
{
    double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
    if (current_duration >= duration)
    {
        return true;
    }
    robot_.target_pose = intepolation(robot_.start_pose, pose, duration, current_duration);
    return false;
}

bool HmMotionBase::move_wrench(geometry_msgs::msg::PoseStamped pose, 
                            geometry_msgs::msg::WrenchStamped wrench, double duration)
{
    double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
    if (current_duration >= duration)
    {
        return true;
    }
    robot_.target_pose = intepolation(robot_.start_pose, pose, duration, current_duration);
    robot_.target_wrench = intepolation_wrench(robot_.current_wrench, wrench, duration, current_duration);
    return false;
}

bool HmMotionBase::sleep(double duration)
{
    double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
    if (current_duration >= duration)
    {
        return true;
    }
    return false;
}

bool HmMotionBase::joint_move(std::vector<double> joints, double time)
{
    if (!system_flags_->service_called)
    {
        system_flags_->service_called = true;
        system_flags_->model_joint = true;
        auto request = std::make_shared<cartesian_controller_msgs::srv::JointMove::Request>();
        request->cmd.data = joints;
        request->duration = time;
        auto result_ = joint_move_client_->async_send_request(request, [this](const rclcpp::Client<cartesian_controller_msgs::srv::JointMove>::SharedFuture motor_future) {
            auto result = motor_future.get();
            // print result
            RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "Joint left send complete");
            RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "success: %d", result->success);
            bool success = result->success;
            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint left Service Failed");
                rclcpp::shutdown();
            }
            
        });
    }

    double current_duration = (system_state_.current_time - system_state_.start_time).seconds();
    if (current_duration >= time+0.5)
    {
        robot_.target_pose = robot_.target_monitor;

        system_flags_->service_called = false;
        system_flags_->model_joint = false;

        return true;
    }
    return false;
}

// Execute the task

void HmMotionBase::task_execute()
{
    auto current_task = tasks_vector_[system_state_.task_num];
    system_state_.current_time = my_clock_.now();
    // The task state for executing the task
    switch (current_task->get_state())
    {
    case HmTaskState::INIT:
    {
        RCLCPP_INFO(this->get_logger(), "Init task [%d]: %s", system_state_.task_num, current_task->get_name().c_str());
        system_flags_->service_called = false;

        robot_.start_pose = robot_.target_monitor;

        system_state_.start_time = my_clock_.now();

        current_task->init();        
        
        current_task->set_state(HmTaskState::EXECUTE);
        break;
    }

    case HmTaskState::EXECUTE:
    {
        current_task->execute();
        break;
    }

    case HmTaskState::FINISH:
    {        

        current_task->set_state(HmTaskState::INIT);
        RCLCPP_INFO(this->get_logger(), "The task [%d]: %s finished", system_state_.task_num, current_task->get_name().c_str());
        system_state_.task_num++;
        break;
    }
    
    default:
        break;
    }
}

void HmMotionBase::task_pushback(std::shared_ptr<hm_research::HmTask> task)
{        
    tasks_vector_.push_back(task);
}

// Start the control loop
void HmMotionBase::start()
{
    control_loop_thread_ = std::thread(&HmMotionBase::control_loop, this);
}

void HmMotionBase::control_loop()
{
    rclcpp::Rate rate(250);
    RCLCPP_INFO(this->get_logger(), "Waiting for initialization");   
    while (rclcpp::ok()) 
    {
        if (check_init())
        {
            task_execute();

            if (!system_flags_->model_joint && active_)
            {
                robot_.target_pose.header.stamp = this->now();
                target_pose_pub_->publish(robot_.target_pose);

                wrench_pub_->publish(robot_.target_wrench);
                robotiq_pub_->publish(robot_.robotiq);
            }
        }

    }
}

} // namespace hm_research
