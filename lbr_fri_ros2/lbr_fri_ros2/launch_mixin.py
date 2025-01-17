from typing import Dict, Optional, Union

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


class LBRFRIROS2Mixin:
    @staticmethod
    def arg_command_guard_variant() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="command_guard_variant",
            default_value="safe_stop",
            description="Command guard variant.",
            choices=["default", "safe_stop"],
        )

    @staticmethod
    def arg_open_loop() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="open_loop",
            default_value="true",
            description="Open loop control. Works best for LBRs. Should only be set to false by experienced users.",
        )

    @staticmethod
    def arg_rt_prio() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rt_prio",
            default_value="80",
            description="Realtime priority of the FRI thread. Realtime kernel required.\n"
            "\tRequires configuration in /etc/security/limits.conf. Add the line:\n"
            "\t'user - rtprio 99', where user is your username.",
        )

    @staticmethod
    def arg_port_id() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="port_id",
            default_value="30200",
            description="Port ID of the FRI communication. Valid in range [30200, 30209].\n"
            "\tUsefull in multi-robot setups.",
        )

    @staticmethod
    def param_command_guard_variant() -> Dict[str, LaunchConfiguration]:
        return {
            "command_guard_variant": LaunchConfiguration(
                "command_guard_variant", default="safe_stop"
            )
        }

    @staticmethod
    def param_open_loop() -> Dict[str, LaunchConfiguration]:
        return {"open_loop": LaunchConfiguration("open_loop", default="true")}

    @staticmethod
    def param_rt_prio() -> Dict[str, LaunchConfiguration]:
        return {"rt_prio": LaunchConfiguration("rt_prio", default="80")}

    @staticmethod
    def param_port_id() -> Dict[str, LaunchConfiguration]:
        return {"port_id": LaunchConfiguration("port_id", default="30200")}

    @staticmethod
    def node_app(
        robot_name: Optional[Union[LaunchConfiguration, str]] = None, **kwargs
    ) -> DeclareLaunchArgument:
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name", default="lbr")
        return Node(
            package="lbr_fri_ros2",
            executable="app",
            namespace=robot_name,
            emulate_tty=True,
            output="screen",
            **kwargs,
        )
