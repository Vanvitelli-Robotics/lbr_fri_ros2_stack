import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from lbr_fri_msgs.msg import LBRState

class FakeLBRNode(Node):
    def __init__(self):
        super().__init__("fake_lbr_node")
        self.joint_state = JointState(
            name=[f"lbr_joint_{i}" for i in range(7)], position=[0.0] * 7
        )
        self.lbr_state = LBRState(sample_time=0.01, measured_joint_position=[0.0]*7)
        self.pub = self.create_publisher(
            JointState, "joint_states", qos_profile=1
        )
        hz = 100
        dt = 1.0 / float(hz)
        self.create_timer(dt, self.callback)

    def callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FakeLBRNode())
    rclpy.shutdown()
