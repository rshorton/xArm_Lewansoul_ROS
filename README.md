# xArm_Lewansoul_ROS
Provides ROS2 integration for xarm robot of Lewansoul.

ROS1 Documentation can be found in the following link: https://xarm-lewansoul-ros.readthedocs.io/en/latest/.

To manually control arm using the Motion Planning plug-in of RViz, run:

    ros2 launch xarm_launch xarm.launch.py

Use the positioning controls to move the arm and then select 'Plan & Execute'.

Tested with ROS2 Galactic (latest updates as of 2/1/22) and source build of moveit2 and ros2_control (https://moveit.picknik.ai/galactic/doc/tutorials/getting_started/getting_started.html)

IKFast is currently used as the kinematics solver and will provide solutions for the RViz use cases.  However, it isn't working well when using the moveit2 motion planning api.  In most cases it will fail to provide a solution.
