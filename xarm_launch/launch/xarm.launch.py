# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF
    robot_description_content = Command(
            ['xacro ', PathJoinSubstitution(
                [
                    FindPackageShare("xarm_description"),
                    "urdf",
                    "xarm.urdf",
                ]
            )]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("xarm_launch"),
            "config",
            "xarm_controllers.yaml",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("xarm_description"), "config", "xarm.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm_forward_position_controller", "-c", "/controller_manager"],
    )

    robot_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm_gripper_forward_position_controller", "-c", "/controller_manager"],
    )

    robot_gripper_3finger_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["xarm_gripper_3finger_trajectory_position_controller", "-c", "/controller_manager"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        #robot_gripper_controller_spawner,
        robot_gripper_3finger_controller_spawner
    ]

    return LaunchDescription(nodes)
