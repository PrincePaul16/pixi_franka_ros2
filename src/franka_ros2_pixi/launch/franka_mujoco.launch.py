# Copyright 2024 Daniel San Jose Pro
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="true -> MuJoCo (crisp_mujoco_sim); false -> real FR3 (franka_hardware).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.16.0.2",
            description="IP of the real robot (only used when use_fake_hardware:=false).",
        )
    )

    start_rviz = LaunchConfiguration("start_rviz")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("franka_ros2_pixi"),
                    "config",
                    "fr3_single.urdf.xacro",
                ]
            ),
            " use_fake_hardware:=", LaunchConfiguration("use_fake_hardware"),
            " robot_ip:=", LaunchConfiguration("robot_ip"),
            " load_gripper:=false",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    pkg_share = get_package_share_directory("franka_ros2_pixi")
    rviz_config_file = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )

    # Use our controllers
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("franka_ros2_pixi"),
            "config",
            "controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[robot_description],
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    controllers = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "cartesian_impedance_controller",
                "--controller-manager",
                "/controller_manager",
                "--inactive",
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "pose_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "twist_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "franka_robot_state_broadcaster",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
    ]

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        *controllers,
    ]

    return LaunchDescription(declared_arguments + nodes)
