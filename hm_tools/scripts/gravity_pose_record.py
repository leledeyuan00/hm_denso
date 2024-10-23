import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, WrenchStamped
from control_msgs.msg import DynamicJointState

from cartesian_controller_msgs.srv import JointMove
import time

setting_joints = [
    [-0.000091, -0.041863, 1.832221, -0.000024, 1.302313, -0.000088],
    [-0.000013, 0.309082, 2.036595, -0.000092, -1.606125, -0.000060],
    [-0.000060, -0.002220, 1.875791, 0.000058, -0.175877, 0.000013],
    [0.000039, -0.002133, 1.875731, 0.000128, -0.175902, -1.529966],
    [-0.008691, -0.023813, 1.676224, 0.044010, 1.040968, -1.578485],
    [-0.367743, 0.122267, 1.665442, 0.819327, 1.315258, -2.277864],
    [0.365346, 0.124186, 1.633914, -0.752863, 1.319552, -0.866864],
]


class MessageRecorder(Node):

    def __init__(self):
        super().__init__("message_recorder")
        self.pose_subscriber = self.create_subscription(PoseStamped, "/cartesian_compliance_controller/current_pose", self.pose_callback, 10)
        self.wrench_subscriber = self.create_subscription(WrenchStamped, "/cartesian_compliance_controller/current_wrench", self.wrench_callback, 10)
        self.wrench_raw_subscriber = self.create_subscription(WrenchStamped, "/cartesian_compliance_controller/ft_sensor_wrench", self.ft_sensor_wrench_callback, 10)

        self.joint_move_client = self.create_client(JointMove, "/cartesian_compliance_controller/target_joint")

        self.current_pose = []
        self.current_wrench = []        

        self.initialized_pose = False
        self.initialized_wrench = False
        self.initialized_wrench_raw = False

        self.pose_file = open("recorded_messages_pose.txt", "w")
        self.wrench_file = open("recorded_messages_wrench.txt", "w")

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.initialized_pose = True
    
    def wrench_callback(self, msg):
        self.current_wrench = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        self.initialized_wrench = True
    
    def ft_sensor_wrench_callback(self, msg):
        self.initialized_wrench_raw = True
    

    def record(self):
        # recorded quaternion as the format: [x, y, z, w]
        self.pose_file.write(f"{ self.current_pose[0]}, {self.current_pose[1]}, {self.current_pose[2]}, {self.current_pose[3]}, {self.current_pose[4]}, {self.current_pose[5]}, {self.current_pose[6]}\n")
        self.wrench_file.write(f"{self.current_wrench[0]}, {self.current_wrench[1]}, {self.current_wrench[2]}, {self.current_wrench[3]}, {self.current_wrench[4]}, {self.current_wrench[5]}\n")
            

    def start_record(self):
        # Wait for all the messages to be initialized
        while not self.initialized_pose or not self.initialized_wrench or not self.initialized_wrench_raw:
            rclpy.spin_once(self)

        for joints in setting_joints:
            duration = 3.0
            joint_move = JointMove.Request()
            joint_move.duration = duration
            joint_move.cmd.data = joints
            future = self.joint_move_client.call_async(joint_move)

            rclpy.spin_until_future_complete(self, future)

            time.sleep(duration + 2.0)
            rclpy.spin_once(self)

            self.record()
            


if __name__ == "__main__":
    rclpy.init()
    node = MessageRecorder()
    node.start_record()
    node.destroy_node()
    rclpy.shutdown()
