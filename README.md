# xArm_Lewansoul_ROS
Provides ROS2 integration for xarm robot of Lewansoul.

ROS1 Documentation can be found in the following link: https://xarm-lewansoul-ros.readthedocs.io/en/latest/.

Run simple test:

In one shell:

    ros2 launch xarm_launch xarm.launch.py

In another shell:

    ros2 launch xarm_test_nodes test_xarm_forward_position_controller.launch.py

The above will open rviz2 with xArm showing, and then the arm will move repeatedly thru 3 poses.

10/21/2021 status

Moveit2 support is a work in progress.  The prebuilt ros2_control and moveit2 packages for Galactic do not appear to be compatible and result in a segfault in the move_group node when using RViz2 to test motion planning via the interactive tools or when using the moveit2 move group demo.  Building moveit2 from source (which pulls-in ros2_control as a dependency) results in the move group demo working, but Rviz2 now crashes when using interactive testing.  Maybe best to revert to Foxy for now?
