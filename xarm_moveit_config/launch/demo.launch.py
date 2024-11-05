import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )


    moveit_config = (
        MoveItConfigsBuilder("xarm")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("xarm_description"),
            "urdf",
            "xarm.urdf")
        )
        .robot_description_semantic(file_path=os.path.join(
            get_package_share_directory("xarm_moveit_config"),
            "config",
            "xarm.srdf")
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )        
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    tutorial_mode = LaunchConfiguration("rviz_tutorial")
    rviz_base = os.path.join(get_package_share_directory("xarm_moveit_config"), "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config, '--ros-args', '--log-level', 'warn'],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,            
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "xarm_base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("xarm_launch"),
        "config",
        "xarm_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        arguments=['--ros-args', '--log-level', 'warn'],
        output="both",
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

    xarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "xarm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    xarm_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "xarm_3finger_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config)
    )

    return LaunchDescription(
        [
            db_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            mongodb_server_node,
            joint_state_broadcaster_spawner,
            xarm_controller_spawner,
            xarm_gripper_controller_spawner,
        ]
    )
