import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import PoseStamped

# Create a node from the class

class JointStateListener(Node):

    def __init__(self):
        super().__init__('joint_state_listener_node')
        self.subscription = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.listener_callback,
            10)
        self.subscription

        self.joints_name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        arm_pose_topic = '/cartesian_compliance_controller/current_pose'

        self.arm_pose_sub = self.create_subscription(
            PoseStamped,
            arm_pose_topic,
            self.arm_pose_callback,
            10)
        

        
        self.left_joints = None
        self.right_joints = None
        self.left_pose = None
        self.right_pose = None

        self.joints_initialized = False
        self.pose_initialized = False


    def listener_callback(self, msg):

        joints_dict = dict(zip(msg.joint_names, msg.interface_values))
        self.joints = [joints_dict[joint_name].values[0] for joint_name in self.joints_name]
        self.joints_initialized = True
    
    def arm_pose_callback(self, msg):
        self.pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.pose_initialized = True
    

    def waiting_for_init(self):
        while not self.joints_initialized or not self.pose_initialized :
            rclpy.spin_once(self)
        
        self.get_logger().info("joints: [%f, %f, %f, %f, %f, %f]" % tuple(self.joints))
        self.get_logger().info("pose: [x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f]" % tuple(self.pose))
    
def main():
    rclpy.init()
    node = JointStateListener()
    node.waiting_for_init()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
