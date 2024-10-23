# Gravity Compensation

Note: The ROS default quaternion is : [x, y, z, w]

## Running the recording program

Be sure that the environment is safety for the robot moving!

1. Launch the ATI sensor. reference to the [ATI ROS driver](https://github.com/smartrobotsdesignlab/ATI_ROS2)

    And please make sure that the sensor's data is stable. While usually it needs prewarm around at least 20-30 min.

2. Launch the robot without force control and as the cartesian controller.

    Note: make sure to disable the force and gravity compensate parameter on the hm_denso_controllers.yaml file!!!!!

    `ros2 launch hm_denso_bringup hm_denso_bringup.launch.py`


3. Be sure that select the correct side

    Go to the correct folder. `xxx/hm_tools/scripts`

    Open the `gravity_pose_record.py` file, and check the 21 row, the "side" parameter.

4. Run the data record python file

    `python3 gravity_pose_record.py`

## Running the gravity computation program

`python3 gravity_compute.py`

The result will be saved at `gravity_result.txt` file.

Then copy the result to the correspoded bringup config yaml file.
