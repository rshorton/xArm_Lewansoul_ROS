# xArm_Lewansoul_ROS+
Provides ROS2 integration for xarm robot of Lewansoul.  Based on the original ROS 1 implementation
by others.

PLEASE NOTE - This branch is for a modified xArm which includes an additional
joint between existing joints 3 and 4 which now makes the arm a true 6 DOF arm and easier to control.

The code has also been revised to use the HiWonder/Lewansoul serial control protocol via
the BusLinker V2.5 interface board to control the servos. (The USB HID interface is still
implemented but untested after the refactoring for support of the serial interface.  
This could not be tested because the original USB interface board died.)

To manually control arm using the Motion Planning plug-in of RViz, run:

    ros2 launch xarm_launch xarm.launch.py

Use the positioning controls to move the arm and then select 'Plan & Execute'.

Example pick and place test app:

* src: xarm_move_group_test/src/move_group_pick.cpp
* video: https://youtu.be/PhVF9UsyZmc

Example usage on moveable robot base:

* https://youtu.be/iXQdU-qKR5s

Tested with ROS2 Galactic as of 3/18/2022.
