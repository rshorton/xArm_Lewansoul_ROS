// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "xarm_hardware_interface/xarm_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#undef FAKE_IT

namespace xarm_hardware
{
  CallbackReturn XArmSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
  {
   if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
   {
     return CallbackReturn::ERROR;
   }


  RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Number of joints= %ld", info_.joints.size());

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_joint_name_.resize(info_.joints.size(), std::string());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_last_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // XArmSystem has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XArmSystemHardware"),
        "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XArmSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XArmSystemHardware"),
        "Joint '%s' has %ld state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("XArmSystemHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Configured");

  return CallbackReturn::SUCCESS;  
}

std::vector<hardware_interface::StateInterface> XArmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
	  hw_joint_name_[i] = info_.joints[i].name;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> XArmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn XArmSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Starting ...please wait...");

#if !defined(FAKE_IT)
  if (!xarm.init()) {
	  return CallbackReturn::ERROR;
  }
#endif

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("XArmSystemHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  // set some default values when starting the first time
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = hw_commands_[i] = xarm.readDefaultPosition(hw_joint_name_[i]);
    hw_commands_last_[i] = std::numeric_limits<double>::max();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("XArmSystemHardware"), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn XArmSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Stopping ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("XArmSystemHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("XArmSystemHardware"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

#if !defined(FAKE_IT)
hardware_interface::return_type XArmSystemHardware::read()
{
  std::vector<double> positions;
  xarm.getAllJointPositions(positions, hw_joint_name_);

  for (uint i = 0; i < hw_states_.size(); i++)
  {
	  if (hw_states_[i] != positions[i])
	  {
		  hw_states_[i] = positions[i];
		  RCLCPP_DEBUG(
			  rclcpp::get_logger("XArmSystemHardware"), "New state %.5f for joint %s (%d)",
			  hw_states_[i], hw_joint_name_[i].c_str(), i);
	  }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XArmSystemHardware::write()
{
  xarm.setAllJointPositions(hw_commands_, hw_joint_name_);
  return hardware_interface::return_type::OK;
}


#else
hardware_interface::return_type XArmSystemHardware::read()
{
  //RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
		//hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
		double new_state = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
	    if ( new_state != hw_states_[i]) {
	    	hw_states_[i] = new_state;
			RCLCPP_DEBUG(
			  rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "New state %.5f for joint %d!",
			  hw_states_[i], i);
	    }
	  }
	//  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XArmSystemHardware::write()
{
  //RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
	  if (hw_states_[i] != hw_commands_[i]) {
		// Simulate sending commands to the hardware
		RCLCPP_INFO(
		  rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "New command %.5f for joint %d",
		  hw_commands_[i], i);
	  }
  }
//  RCLCPP_INFO(
//    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}
#endif // FAKE_IT

}  // namespace xarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
		xarm_hardware::XArmSystemHardware, hardware_interface::SystemInterface)
