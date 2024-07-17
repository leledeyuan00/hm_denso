#include "hm_motion_test/hm_motion_fsm.hpp"

using namespace std::chrono_literals;

HmMotionFsm::HmMotionFsm() : Node("hm_motion_fsm")
{
    ros_init();
    tasks_init();
}


void HmMotionFsm::ros_init()
{
    
    my_clock_ = rclcpp::Clock(RCL_STEADY_TIME);
    start_time_ = my_clock_.now();
    // publishers
    target_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/cartesian_compliance_controller/target_frame", rclcpp::SystemDefaultsQoS());
    wrench_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>("/cartesian_compliance_controller/target_wrench", rclcpp::SystemDefaultsQoS());
    // subscribers
    current_ee_tf_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cartesian_compliance_controller/current_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            current_ee_tf_ = *msg;
            if (!initialized_)
            {
                target_pose_ = *msg;
                home_ee_tf_ = *msg;
                start_time_ = my_clock_.now();
                initialized_ = true;
            }
        });
    current_target_monitor_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cartesian_compliance_controller/target_frame_monitor", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            target_monitor_ = *msg;
        });

    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/cartesian_compliance_controller/current_wrench", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            current_wrench_ = *msg;
        });

    // clients
    test_client_ = this->create_client<std_srvs::srv::SetBool>("test_service");


    joint_move_client_ = this->create_client<cartesian_controller_msgs::srv::JointMove>("/cartesian_compliance_controller/target_joint");

    // get parameters
    // get projection matrix from parameter server
    std::vector<double> robot_camera_trans, robot_camera_quat;
    // Declare parameters that could be set on this node
    this->declare_parameter("robot_camera.translation", std::vector<double>{0.0, 0.0, 0.0});
    if (!this->get_parameter("robot_camera.translation", robot_camera_trans))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter robot_camera.translation");
    }
    
    this->declare_parameter("robot_camera.quaternion", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (!this->get_parameter("robot_camera.quaternion", robot_camera_quat))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter robot_camera.rotation");
    }

    Eigen::Quaterniond robot_camera_q(robot_camera_quat[0], robot_camera_quat[1], robot_camera_quat[2], robot_camera_quat[3]);
    calibration_matrix_ = Eigen::Isometry3d(
        Eigen::Translation3d(robot_camera_trans[0], robot_camera_trans[1], robot_camera_trans[2]) *
        (robot_camera_q ));
    
    std::cout << "calibration_matrix_:" << std::endl << calibration_matrix_.matrix() << std::endl;
}


geometry_msgs::msg::PoseStamped HmMotionFsm::intepolation(geometry_msgs::msg::PoseStamped &init_pose, geometry_msgs::msg::PoseStamped target_pose,
                        double current_duration, double duration)
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

geometry_msgs::msg::WrenchStamped HmMotionFsm::intepolation_wrench(geometry_msgs::msg::WrenchStamped &init_wrench, geometry_msgs::msg::WrenchStamped target_wrench,
                        double current_duration, double duration)
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

void HmMotionFsm::task_sleep(rclcpp::Time start_time, double duration)
{
    double current_duration = (my_clock_.now() - start_time).seconds();
    if (current_duration >= duration)
    {
        system_state_++;
        new_task_ = false;
    }    
}

void HmMotionFsm::task_intepolation_with_wrench(geometry_msgs::msg::PoseStamped pose, geometry_msgs::msg::WrenchStamped wrench, double duration)
{
    double current_duration = (my_clock_.now() - start_time_).seconds();
    target_pose_ = intepolation(target_temp_pose_, pose, current_duration, duration);
    target_wrench_ = intepolation_wrench(target_temp_wrench_,  wrench, current_duration, duration);
    if (current_duration >= duration)
    {
        target_temp_wrench_ = target_wrench_;
        system_state_++;
        new_task_ = false;
    }    
}

void HmMotionFsm::task_intepolation(geometry_msgs::msg::PoseStamped pose, double duration)
{
    double current_duration = (my_clock_.now() - start_time_).seconds();
    target_pose_ = intepolation(target_temp_pose_, pose, current_duration, duration);
    if (current_duration >= duration)
    {
        system_state_++;
        new_task_ = false;
    }    
}

void HmMotionFsm::test_service(bool on)
{
    if (!service_called_)
    {
        service_called_ = true;
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = on;
        auto result = test_client_->async_send_request(request, [this](const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture test_future) {
            auto result = test_future.get();
            // print result
            RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "Test Service Result:");
            RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "success: %d", result->success);
            bool success = result->success;
            if (success)
            {
                service_called_ = false; // reset the flag
                system_state_++;
                new_task_ = false;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Test Service Failed");
                rclcpp::shutdown();
            }
            
        });
    }
}


void HmMotionFsm::task_joint_move(std::vector<double> joints, double time)
{
    if (!service_called_)
    {
        service_called_ = true;
        model_joint_ = true;
        auto request = std::make_shared<cartesian_controller_msgs::srv::JointMove::Request>();
        request->cmd.data = joints;
        request->duration = time;

        auto result = joint_move_client_->async_send_request(request, [this](const rclcpp::Client<cartesian_controller_msgs::srv::JointMove>::SharedFuture motor_future) {
            auto result = motor_future.get();
            // print result
            RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "Joint send complete");
            RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "success: %d", result->success);
            bool success = result->success;
            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint Service Failed");
                rclcpp::shutdown();
            }
            
        });
    }

    double current_duration = (my_clock_.now() - start_time_).seconds();
    if (current_duration >= time+0.5)
    {
        target_pose_ = current_ee_tf_;
        target_wrench_ = current_wrench_;
        service_called_ = false;
        model_joint_ = false;
        system_state_++;
        new_task_ = false;
    }  
}

void HmMotionFsm::tasks_init()
{

    // Go Home
    tasks_vector_.push_back(
        [&](void){
            if (!new_task_)
            {
                RCLCPP_INFO(this->get_logger(), "Go home");
                new_task_ = true;
                start_time_ = my_clock_.now();
                service_called_ = false;
            }
            std::vector<double> home_joints = {0.0, 0.253510, 1.732042, 0.0, 1.107231, 0.0};
            task_joint_move( home_joints, 5.0);
        }
    );

    // Move up
    tasks_vector_.push_back(
        [&](void){
            if (!new_task_)
            {
                RCLCPP_INFO(this->get_logger(), "Moving up");
                new_task_ = true;
                start_time_ = my_clock_.now();
                target_temp_pose_ = target_monitor_;
            }
            geometry_msgs::msg::PoseStamped pose;

            pose = target_temp_pose_;

            pose.pose.position.z += 0.1;

            task_intepolation(pose, 1.0);
        }
    );

    // Move forward
    tasks_vector_.push_back(
        [&](void){
            if (!new_task_)
            {
                RCLCPP_INFO(this->get_logger(), "Moving forward");
                new_task_ = true;
                start_time_ = my_clock_.now();
                target_temp_pose_ = target_monitor_;
            }
            geometry_msgs::msg::PoseStamped pose;

            pose = target_temp_pose_;

            pose.pose.position.x += 0.1;

            task_intepolation(pose, 1.0);
        }
    );

    // Move down
    tasks_vector_.push_back(
        [&](void){
            if (!new_task_)
            {
                RCLCPP_INFO(this->get_logger(), "Moving down");
                new_task_ = true;
                start_time_ = my_clock_.now();
                target_temp_pose_ = target_monitor_;
            }
            geometry_msgs::msg::PoseStamped pose;

            pose = target_temp_pose_;

            pose.pose.position.z -= 0.1;

            task_intepolation( pose, 1.0);
        }
    );

    // Move back
    tasks_vector_.push_back(
        [&](void){
            if (!new_task_)
            {
                RCLCPP_INFO(this->get_logger(), "Moving back");
                new_task_ = true;
                start_time_ = my_clock_.now();
                target_temp_pose_ = target_monitor_;
            }
            geometry_msgs::msg::PoseStamped pose;

            pose = target_temp_pose_;

            pose.pose.position.x -= 0.1;

            task_intepolation( pose, 1.0);
        }
    );

    // Shut down
    tasks_vector_.push_back(
        [&](void){
            RCLCPP_INFO(this->get_logger(), "Finished");
            rclcpp::shutdown();
        }
    );
}

void HmMotionFsm::tasks_execute()
{
    tasks_vector_[system_state_]();
}


// Start the control loop
void HmMotionFsm::start()
{
    control_loop_thread_ = std::thread(&HmMotionFsm::control_loop, this);
}

// Control loop
void HmMotionFsm::control_loop()
{
    rclcpp::Rate rate(125);
    RCLCPP_INFO(this->get_logger(), "Waiting for initialization");    
    while (rclcpp::ok())
    {
        if (initialized_)
        {   
            tasks_execute();   
            if (!model_joint_)
            {
                target_pose_.header.stamp = this->now();
                target_pose_pub_->publish(target_pose_);
                current_ee_tf_.header.stamp = this->now();
                wrench_pub_->publish(target_wrench_);
            }            
        }
        rate.sleep();
    }
}


int main(int argc, char const *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "HM Motion Test by FSM Node Started");
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<HmMotionFsm>();

    node->start();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "HM Motion Test by FSM Node Stopped");
    return 0;
}
