import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from lbr_fri_msgs.msg import LBRState


class RemapNode(Node):
    def __init__(self):
        super().__init__("joint_state_remap_node")
        self.declare_parameter("robot_name", "lbr")
        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.joint_state = JointState(name=[f"{robot_name}_joint_{i}" for i in range(7)])
        self.pub = self.create_publisher(JointState, "joint_states", qos_profile=1)
        self.sub = self.create_subscription(
            LBRState, "lbr_state", self.callback, qos_profile=1
        )

    def callback(self, lbr_state):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = lbr_state.measured_joint_position.tolist()
        self.pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RemapNode())
    rclpy.shutdown()
