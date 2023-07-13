import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from lbr_fri_msgs.msg import LBRCommand, LBRState

from .admittance_controller import AdmittanceController

np.set_printoptions(precision=3, suppress=True, linewidth=1000)


class ObstacleListener:
    def __init__(self, node):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        self.obs = [None] * 8

        hz = 100
        dt = 1.0 / float(hz)
        self.timer = node.create_timer(dt, self.callback)

    def get_position(self, link):
        link_name = f"static_lbr_link_{link}"
        try:
            t = self.tf_buffer.lookup_transform("world", link_name, rclpy.time.Time())
        except TransformException as ex:
            return None
        transl = t.transform.translation
        return [transl.x, transl.y, transl.z]

    def callback(self):
        for link in range(7):
            self.obs[link] = self.get_position(link)
        self.obs[-1] = self.get_position("ee")

    def ready(self):
        return all(o is not None for o in self.obs)

    def get_obstacles(self):
        return np.array(self.obs).T


class EndEffectorWrenchEstimator:
    def __init__(self, Nmax, smooth, end_effector_link, robot_model):
        self.jac = robot_model.get_global_link_geometric_jacobian_function(
            end_effector_link, numpy_output=True
        )
        self.data = []
        self.Nmax = Nmax
        self.smooth = smooth
        self.offset = None
        self.wrench_estimate = np.zeros(6)

    def append(self, joint_position, external_torque):
        ndata = len(self.data)
        if ndata < self.Nmax:
            # self.data.append(self.estimate_wrench(joint_position, external_torque))
            self.data.append(external_torque)
        elif ndata == self.Nmax:
            self.offset = np.mean(self.data, axis=0)

    def ready(self):
        return self.offset is not None

    def estimate_wrench(self, joint_position, external_torque):
        J = self.jac(joint_position)
        Jinv = np.linalg.pinv(J, rcond=0.05)
        tau_ext = np.asarray(external_torque)
        f_ext = Jinv.T @ tau_ext
        return f_ext.tolist()

    def estimate_offset_wrench(self, joint_position, external_torque):
        # Estimate offset external torque
        external_torque = np.array(external_torque) - self.offset

        # Estimate wrench from raw joint state
        wrench_estimate = np.array(
            self.estimate_wrench(joint_position, external_torque)
        )

        # Remove offset
        # wrench_estimate -= self.offset

        # Smooth estimate
        self.wrench_estimate = (
            self.smooth * wrench_estimate + (1.0 - self.smooth) * self.wrench_estimate
        )

        return self.wrench_estimate.tolist()


class AdmittanceControlNode(Node):
    def __init__(self, node_name="admittance_control_node") -> None:
        super().__init__(node_name=node_name)

        # parameters
        self.declare_parameter("robot_description", "")
        self.declare_parameter("end_effector_link", "lbr_link_ee")

        # Setup controller
        end_effector_link = str(self.get_parameter("end_effector_link").value)
        robot_description = str(self.get_parameter("robot_description").value)
        self.controller = AdmittanceController(
            self,
            robot_description,
            end_effector_link,
        )

        # Setup obstacle listener
        self.obs_listener = ObstacleListener(self)

        # Setup wrench estimator
        Nmax = 500
        smooth = 0.1
        self.wrench_estimator = EndEffectorWrenchEstimator(
            Nmax, smooth, end_effector_link, self.controller.robot_model
        )
        self.reported_ready = False

        # Setup joint position
        self.joint_position = None

        # publishers and subscribers
        self.lbr_state_sub_ = self.create_subscription(
            LBRState,
            "/lbr_state",
            self.on_lbr_state_,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                deadline=Duration(nanoseconds=10 * 1e6),  # 10 milliseconds
            ),
        )
        self.lbr_command_pub_ = self.create_publisher(
            LBRCommand,
            "/lbr_command",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                deadline=Duration(nanoseconds=10 * 1e6),  # 10 milliseconds
            ),
        )

    def on_lbr_state_(self, lbr_state: LBRState) -> None:
        # Check obstacle listener
        if not self.obs_listener.ready():
            return

        # Extract joint state
        q, tau_ext = lbr_state.measured_joint_position, lbr_state.external_torque
        if self.joint_position is None:
            self.joint_position = np.array(q)

        # Update wrench estimator (return if not ready)
        self.wrench_estimator.append(q, tau_ext)

        if not self.wrench_estimator.ready():
            return

        if not self.reported_ready:
            self.get_logger().info("Controller ready to use!")
            self.reported_ready = True

        # Retrieve wrench estimate
        f_ext = np.array(self.wrench_estimator.estimate_offset_wrench(q, tau_ext))
        self.get_logger().info("f_ext=" + repr(f_ext))

        # Retrieve current joint state (overwrite q from lbr_state)
        q = self.joint_position.copy()

        # Retrieve sample time
        dt = lbr_state.sample_time

        # Retrieve obstacles
        obs = self.obs_listener.get_obstacles()

        # Compute goal joint position using admittance controller
        qg = self.controller(q, f_ext, dt, obs)

        # Send command
        lbr_command = LBRCommand()
        lbr_command.joint_position = qg.tolist()
        self.lbr_command_pub_.publish(lbr_command)

        dq = (qg - q) / dt
        self.get_logger().info("dq=" + repr(dq))
        # self.get_logger().info("qg=" + repr(qg))

        # Update joint position
        self.joint_position = qg.copy()


def main(args=None):
    rclpy.init(args=args)
    admittance_control_node = AdmittanceControlNode()
    rclpy.spin(admittance_control_node)
    rclpy.shutdown()
