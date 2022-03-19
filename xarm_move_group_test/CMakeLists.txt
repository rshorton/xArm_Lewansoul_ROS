cmake_minimum_required(VERSION 3.5)
project(xarm_move_group_test)


find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
#find_package(rviz_visual_tools REQUIRED)
#find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    rclcpp_action
    tf2_geometry_msgs
    tf2_ros
    moveit_core
    #rviz_visual_tools
    #moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    moveit_ros_planning
    pluginlib
    Boost
)

#add_executable(xarm_move_group_test
#    src/move_group_test.cpp)

add_executable(xarm_move_group_pick
    src/move_group_pick.cpp)

#ament_target_dependencies(xarm_move_group_test
#    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

ament_target_dependencies(xarm_move_group_pick
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

include_directories(./thirdparty/nlohmann-json)

#install(TARGETS xarm_move_group_test
#    DESTINATION lib/${PROJECT_NAME}
#)

install(TARGETS xarm_move_group_pick
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

