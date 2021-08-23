# xArm_Lewansoul_ROS
Provides ROS2 integration for xarm robot of Lewansoul.

ROS1 Documentation can be found in the following link: https://xarm-lewansoul-ros.readthedocs.io/en/latest/.

Run simple test:

In one shell:

    ros2 launch xarm_launch xarm.launch.py

In another shell:

    ros2 launch xarm_test_nodes test_xarm_forward_position_controller.launch.py

The above will open rviz2 with xArm showing, and then the arm will move repeatedly thru 3 poses.
