cmake_minimum_required(VERSION 3.5)
project(xarm_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

pkg_check_modules(hidapi-hidraw REQUIRED IMPORTED_TARGET hidapi-hidraw)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/xarm_system.cpp
  src/xarm.cpp
  #src/xarm_usb.cpp
  src/xarm_serial.cpp
  src/serial_servo_bus.cpp
)

add_executable(
  "test_drvr"
  src/test.cpp
  src/xarm_serial.cpp
  src/serial_servo_bus.cpp
)

target_include_directories(
  "test_drvr"
  PRIVATE
  include
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)


ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  hidapi-hidraw
)

ament_target_dependencies(
  "test_drvr"
  hardware_interface
  rclcpp
)

target_link_libraries(${PROJECT_NAME} PkgConfig::hidapi-hidraw)

pluginlib_export_plugin_description_file(hardware_interface xarm_hardware.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  TARGETS "test_drvr"
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)
ament_package()
