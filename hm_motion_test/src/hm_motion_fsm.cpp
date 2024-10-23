#include "hm_motion_test/hm_motion_fsm.hpp"


using namespace std::chrono_literals;

namespace hm_research
{

HmMotionFsm::HmMotionFsm() : HmMotionBase("hm_motion_fsm")
{
    custom_init();
    tasks_init();
}

// For custom initialization such as service client and parameters
void HmMotionFsm::custom_init()
{
    // test service
    test_client_ = this->create_client<std_srvs::srv::SetBool>("/test_service");

    // get parameters
    // get projection matrix from parameter server
    std::vector<double> robot_camera_trans, robot_camera_quat;
    // Declare parameters that could be set on this node
    this->declare_parameter("robot_camera_r.translation", std::vector<double>{0.0, 0.0, 0.0});
    if (!this->get_parameter("robot_camera_r.translation", robot_camera_trans))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter robot_camera_r.translation");
    }
    
    this->declare_parameter("robot_camera_r.quaternion", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (!this->get_parameter("robot_camera_r.quaternion", robot_camera_quat))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter robot_camera_r.rotation");
    }

    Eigen::Quaterniond robot_camera_q(robot_camera_quat[0], robot_camera_quat[1], robot_camera_quat[2], robot_camera_quat[3]);
    calibration_matrix_r_ = Eigen::Isometry3d(
        Eigen::Translation3d(robot_camera_trans[0], robot_camera_trans[1], robot_camera_trans[2]) *
        (robot_camera_q ));
    
    std::cout << "calibration_matrix_r_:" << std::endl << calibration_matrix_r_.matrix() << std::endl;
}



void HmMotionFsm::tasks_init()
{

    // Go Home
    task_pushback(TaskPtr("Go Home", [this](){
        std::vector<double> home_joints = {0.0, 0.253510, 1.732042, 0.0, 1.107231, 0.0};

        if(joint_move(home_joints, 5.0)){
            set_task_finished();
        }
    }));

    // Move up
    task_pushback(TaskPtr("Move up", [this](){
        geometry_msgs::msg::PoseStamped pose;
        pose = get_robot_state().start_pose;

        pose.pose.position.z += 0.1;

        if (move(pose, 1.0)){
            set_task_finished();
        }
    }));

    // Move forward
    task_pushback(TaskPtr("Move forward", [this](){
        geometry_msgs::msg::PoseStamped pose;
        pose = get_robot_state().start_pose;

        pose.pose.position.x += 0.1;

        if (move(pose, 1.0)){
            set_task_finished();
        }
    }));

    // Move down
    task_pushback(TaskPtr("Move down", [this](){
        geometry_msgs::msg::PoseStamped pose;
        pose = get_robot_state().start_pose;

        pose.pose.position.z -= 0.1;

        if (move(pose, 1.0)){
            set_task_finished();
        }
    }));

    // Move back
    task_pushback(TaskPtr("Move back", [this](){
        geometry_msgs::msg::PoseStamped pose;
        pose = get_robot_state().start_pose;

        pose.pose.position.x -= 0.1;

        if (move(pose, 1.0)){
            set_task_finished();
        }
    }));

    // Log Test
    task_pushback(TaskPtr("Log Test", [this](){
        RCLCPP_INFO(this->get_logger(), "Log Test");
        auto robot = get_robot_state();
        auto system_state = get_system_state();

        RCLCPP_INFO(this->get_logger(), "Left current Pose is: [%f, %f, %f]", robot.current_pose.pose.position.x, robot.current_pose.pose.position.y, robot.current_pose.pose.position.z);

        RCLCPP_INFO(this->get_logger(), "Current system task number is: %d", system_state.task_num);
        RCLCPP_INFO(this->get_logger(), "Current system start time is: %f", system_state.start_time.seconds());
        RCLCPP_INFO(this->get_logger(), "Current system current time is: %f", system_state.current_time.seconds());

        set_task_finished();
    }));

    // An example for task with init function and sending the target pose command in the loop
    task_pushback(TaskPtr("Custom Test", 
    // init function
    [this](){
        RCLCPP_INFO(this->get_logger(), "This function will be called only once when the task is initialized. You could do some initialization here.");
    },
    // execute function
    [this](){
        geometry_msgs::msg::PoseStamped pose;
        pose = get_robot_state().start_pose;

        double current_duration = (get_system_state().current_time - get_system_state().start_time).seconds();

        pose.pose.position.z += current_duration * 0.02;

        if (current_duration <= 5.0)
        {
            set_target_pose(pose);
        }
        else
        {
            set_task_finished();
        }
    }));

    // // Shut down
    task_pushback(TaskPtr("Shut down", [this](){
        //initialize the task
        rclcpp::shutdown();
    }));
}


} // namespace hm_research

int main(int argc, char const *argv[])
{
    RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "Hm Motion Test by FSM Node Started");
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<hm_research::HmMotionFsm>();

    node->start();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("hm_motion_fsm"), "Hm Motion Test by FSM Node Stopped");
    return 0;
}
