import os
from math import radians

import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext) -> LaunchDescription:
    model = LaunchConfiguration("model").perform(context)

    urdf = os.path.join(
        get_package_share_directory("lbr_description"),
        "urdf",
        model,
        f"{model}.urdf.xacro",
    )

    robot_description = xacro.process(
        urdf,
        mappings={"sim": "false"},
    )

    admittance_control_node = Node(
        package="lbr_fri_ros2_advanced_python_demos",
        executable="admittance_control_node",
        # output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_remap = Node(
        package="lbr_fri_ros2_advanced_python_demos",
        executable="joint_state_remap",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        arguments=[urdf],
    )

    static_robot_description = xacro.process(
        urdf,
        mappings={"sim": "false", "world_name": "static_world", "robot_name": "static_lbr"},
    )

    static_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="static_robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": static_robot_description, "use_sim_time": False}
        ],
        arguments=[urdf],
        remappings=[
            ("/joint_states", "/static/joint_states"),
            ("/robot_description", "/static/robot_description"),
        ],
    )

    rviz = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("lbr_fri_ros2_advanced_python_demos"),
                "launch",
                "rviz.rviz",
            )
        ],
    )

    visualization = Node(
        package="lbr_fri_ros2_advanced_python_demos",
        executable="visualization",
    )

    trolly_to_world_static_transform_broadcaster = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", str(radians(30)), "0", "trolly", "world"],
    )

    nodes = [
        admittance_control_node,
        joint_state_remap,
        robot_state_publisher,
        rviz,
        visualization,
        trolly_to_world_static_transform_broadcaster,
        static_robot_state_publisher,
    ]

    fake = bool(LaunchConfiguration("fake").perform(context).capitalize())
    if fake:
        fake_joint_states = Node(
            package="lbr_fri_ros2_advanced_python_demos",
            executable="fake_lbr",
        )

        world_to_static_robot_base_static_transform_broadcaster = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["1", "0", "0", str(radians(-30)), "0", "0", "world", "static_world"],
        )

        nodes.append(fake_joint_states)
        nodes.append(world_to_static_robot_base_static_transform_broadcaster)

    return nodes


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="med7",
        description="The LBR model in use.",
        choices=["iiwa7", "iiwa14", "med7", "med14"],
    )

    fake_arg = DeclareLaunchArgument(
        name="fake",
        default_value="true",
        description="Whether to faket the static robot.",
        choices=["true", "false"],
    )
    return LaunchDescription(
        [model_arg, fake_arg, OpaqueFunction(function=launch_setup)]
    )
