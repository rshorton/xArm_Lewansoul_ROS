# xArm_Lewansoul_ROS+
Provides ROS2 integration for xarm robot of Lewansoul.

PLEASE NOTE - This branch is for a modified xArm which includes an
additional joint between existing joints 3 and 4.  

ROS1 Documentation can be found in the following link: https://xarm-lewansoul-ros.readthedocs.io/en/latest/.

To manually control arm using the Motion Planning plug-in of RViz, run:

    ros2 launch xarm_launch xarm.launch.py

Use the positioning controls to move the arm and then select 'Plan & Execute'.

Tested with ROS2 Galactic (latest updates as of 2/1/22) and source build of moveit2 and ros2_control (https://moveit.picknik.ai/galactic/doc/tutorials/getting_started/getting_started.html)

