# hm_denso
The denso robot repository for Hashimoto sensei Lab. It contains controller configuration files and description urdf files. There is also an demo for controlling the robot.

## Dependency
This package is based on the Densor robot package. [The ROS2 version can be found here.](https://github.com/smartrobotsdesignlab/denso_robot_ros2)

Cartesian Controller is based on the [Cartesian Controller](https://github.com/smartrobotsdesignlab/cartesian_controllers)

The Force sensor package is based on the [ATI ROS driver](https://github.com/smartrobotsdesignlab/ATI_ROS2)

## Launch Robot

### Gazebo

`ros2 launch hm_denso_bringup hm_denso_bringup.launch.py sim_gazebo:=true`

### Real Robot

`ros2 launch hm_denso_bringup hm_denso_bringup.launch.py`

## Demo 

`ros2 launch hm_motion_test hm_motion_fsm.launch.py`

### Controlling by Moveit

`./switch_trajectory_controllers.bash`

`ros2 launch hm_denso_moveit_config hm_denso_moveit.launch.py use_sim_time:=true`

### Controlling by Cartesian Controller (Default)

`./switch_cartesian_controllers.bash`


## Guide for Next step

[Guide for Next Setp](./GUIDE_INITIAL.md)