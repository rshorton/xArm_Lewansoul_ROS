#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>     

#include "rclcpp/rclcpp.hpp"
#include "xarm_hardware_interface/xarm.h"

#include "xarm_hardware_interface/xarm_serial.hpp"
#include "xarm_hardware_interface/xarm_usb.hpp"

#define MAX_STR 255
#define PI 3.14159265359
#define INVALID_POS 99999	// Invalid servo value

//#define XARM_USB

//const int UPDATE_PERIOD_MS = 30;
const int UPDATE_PERIOD_MOVING_MS = 20;
const int UPDATE_PERIOD_IDLE_MS = 100;

// Use the idle update period if this many 'moving' update
// periods occur without getting a move command.
const int IDLE_ENTRY_CNT = 50;

// How often to check for the file that indicates the control loop should be
// in manual mode where the user can manually move the robot arm (for specifying
// positions for training.)
const int UPDATE_CNT_CHK_FOR_MANUAL_MODE = (2000/UPDATE_PERIOD_IDLE_MS);
// File to create to enable the manual mode
const std::string MANUAL_MODE_ENABLE_FILE = "/tmp/xarm_enable_manual_mode";

// See comments below
const int SMALL_CHANGE = 20;
const int LARGE_CHANGE_MOVE_TIME = 1500;

const int NUM_JOINTS = 7;

const std::string SERIAL_DEV = "/dev/servo_driver";

namespace xarm
{
	xarm::xarm():
		inited_(false),
		run_(false),
		gripper_pos_min_m_(0.0),
		gripper_pos_min_s_(0.0),
		gripper_pos_max_s_(0.0),
		gripper_pos_m_to_s_factor_(0.0),
		new_cmd_(false)
	{
	}

	xarm::~xarm()
	{
		if (inited_) {
			run_ = false;
			thread_.join();
		}

		if (drvr_) {
			drvr_->close();
		}
	}

	bool xarm::init()
	{
		if (inited_) {
			return false;
		}

		std::string dev;
#if defined(XARM_USB)
		drvr_ = std::make_unique<xarm_usb>();
#else
		drvr_ = std::make_unique<xarm_serial>();
		dev = SERIAL_DEV;
#endif
		if (!drvr_) {
			return false;
		}				

		if (!drvr_->open(dev)) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Failed to open driver");
			return false;
		}

		//Dictionary of joint_names to joint_id
		joint_name_map_.insert(std::make_pair("xarm_1_joint" , 1));
		joint_name_map_.insert(std::make_pair("xarm_1_joint_mirror" , 11));
		joint_name_map_.insert(std::make_pair("xarm_2_joint" , 2));
		joint_name_map_.insert(std::make_pair("xarm_3_joint" , 3));
  		joint_name_map_.insert(std::make_pair("xarm_4_joint" , 4));
		joint_name_map_.insert(std::make_pair("xarm_5_joint" , 5));
		joint_name_map_.insert(std::make_pair("xarm_6_joint" , 6));
		joint_name_map_.insert(std::make_pair("xarm_7_joint" , 7));

		// gripper range servo units:             700   -  200
		// corresponding to phy units of meters:  0.003 - 0.028
		// 0.3 is 1/2 the total grip width since mimic joint used in urdf
		gripper_pos_min_m_ = 0.003; // meters
		// Fix - cleanup, 700 drives the servo to the mechanical limit
		// of the grabber and causes the servo motor to overheat.
		// 650 seems like a same max.  What does that make the physical range?
//		gripper_pos_min_s_ = 700.0; // servo units
		gripper_pos_min_s_ = 650.0; // servo units
		gripper_pos_max_s_ = 200.0;
								  // scale factor: mult scale by phy units in meter to get servo units
		gripper_pos_m_to_s_factor_ = (gripper_pos_max_s_ - gripper_pos_min_s_)/(0.028 - gripper_pos_min_m_);

		joint_range_limits_["xarm_2_joint"][0] = 200;	// min in servo units
		joint_range_limits_["xarm_2_joint"][1] = 980;	// max in servo units
		joint_range_limits_["xarm_2_joint"][2] = 1;	// -1 to invert range
		joint_range_limits_["xarm_3_joint"][0] = 140;
		joint_range_limits_["xarm_3_joint"][1] = 880;
		joint_range_limits_["xarm_3_joint"][2] = -1;
		joint_range_limits_["xarm_4_joint"][0] = 870;
		joint_range_limits_["xarm_4_joint"][1] = 130;
		joint_range_limits_["xarm_4_joint"][2] = -1;
		joint_range_limits_["xarm_5_joint"][0] = 140;
		joint_range_limits_["xarm_5_joint"][1] = 880;
		joint_range_limits_["xarm_5_joint"][2] = -1;
		joint_range_limits_["xarm_6_joint"][0] = 90;
		joint_range_limits_["xarm_6_joint"][1] = 845;
		joint_range_limits_["xarm_6_joint"][2] = 1;
		joint_range_limits_["xarm_7_joint"][0] = 85;
		joint_range_limits_["xarm_7_joint"][1] = 846;
		joint_range_limits_["xarm_7_joint"][2] = 1;

		RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Joint limits:");

		for (const auto &j: joint_name_map_) {
			const auto &name = j.first;
			last_pos_set_map_[name] = INVALID_POS;
			last_pos_get_map_[name] = INVALID_POS;

			// Print ranges in radians
			RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Joint: %s,  min,max:  %f, %f",
				name.c_str(), 
				jointValueToPosition(name, joint_range_limits_[name][0]),
				jointValueToPosition(name, joint_range_limits_[name][1]));
		}

		run_ = true;
		thread_ = std::thread{std::bind(&xarm::Process, this)};

		inited_ = true;
		return true;
	}

	// Set position of all joint positions.  Any changes to the positions will be applied on the next
	// periodic update.  Any previously specified update position that has not be applied yet will be
	// dropped.
	void xarm::setAllJointPositions(const std::vector<double> &commands, const std::vector<std::string> &joints)
	{
		std::lock_guard<std::mutex> guard(mutex_);
	 	for (uint i = 0; i < commands.size(); i++) {
			const std::string &name = joints[i];
			if (name == "xarm_dummy_joint") {
				RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "New pos cmd %*s %s: %.5f",
							i*8, "",
							name.c_str(),
							commands[i]);
				continue;
			}

			int joint_pos = positionToJointValue(name, commands[i]);
			if (joint_pos != last_pos_set_map_[name]) {
				RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "New pos cmd %*s %s: %.5f",
							i*8, "",
							name.c_str(),
							commands[i]);
				last_pos_set_map_[name] = joint_pos;
				// Run in open-loop while moving by immediately reporting the movement has completed
				// since reading the actual position from the servos during motion causes too much
				// delay and jerky motion as a result.  Once motion stops, the actual joint positions
				// will updated by the update thread.
				last_pos_get_map_[name] = joint_pos;
				new_cmd_ = true;
			}
		}
	}

	// Get position of all joints.  The returned position vector corresponds to the last periodic update.
	void xarm::getAllJointPositions(std::vector<double> &positions, const std::vector<std::string> &joints)
	{
		std::lock_guard<std::mutex> guard(mutex_);
	 	for (uint i = 0; i < joints.size(); i++) {
			if (joints[i] == "xarm_dummy_joint") {
				positions.push_back(0.0);	
			} else {
				positions.push_back(jointValueToPosition(joints[i], last_pos_get_map_[joints[i]]));
				RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Get cur pos %*s %s: %.5f",
							(i - 1)*8, "",
							joints[i].c_str(),
							positions[i]);
			}				
		}
	}

	int xarm::convertRadToUnit(std::string joint_name, double rad)
	{
		// Range in servo units
		double range = joint_range_limits_[joint_name][1] - joint_range_limits_[joint_name][0];
		// Mid-range in servo units
		//double b = joint_range_limits_[joint_name][0][1] - range/2;
		double b = 500.0;
		return (range*rad/PI*joint_range_limits_[joint_name][2]) + b;
	}

	double xarm::convertUnitToRad(std::string joint_name, int unit)
	{
		// Range in servo units
		double range = joint_range_limits_[joint_name][1] - joint_range_limits_[joint_name][0];
		// Mid-range in servo units
		double b = 500.0;
		//double b = joint_range_limits_[joint_name][0][1] - range/2;
		return (unit - b)*PI*joint_range_limits_[joint_name][2]/range;
	}

	double xarm::jointValueToPosition(std::string joint_name, int jointValue)
	{
		double position = 0.0;

		if (joint_name == "xarm_1_joint" || joint_name == "xarm_1_joint_mirror") {
			float pos = (float)jointValue;
			if (pos > gripper_pos_min_s_) {
				pos = gripper_pos_min_s_;
			} else if (pos < gripper_pos_max_s_) {
				pos = gripper_pos_max_s_;
			}
			position = (pos - gripper_pos_min_s_)/gripper_pos_m_to_s_factor_ + gripper_pos_min_m_;
			if (joint_name == "xarm_1_joint_mirror") {
				position *= -1;
			}
		} else {
			position = convertUnitToRad(joint_name, jointValue);
		}
		return position;
	}

	int xarm::positionToJointValue(std::string joint_name, double position)
	{
		int position_unit = 0;

		if (joint_name == "xarm_1_joint") {
			double pos_in = position;
			if (pos_in < gripper_pos_min_m_) {
				pos_in = gripper_pos_min_m_;
			}
			position_unit = (int)((pos_in - gripper_pos_min_m_)*gripper_pos_m_to_s_factor_ + gripper_pos_min_s_);

			if (position_unit < gripper_pos_max_s_) {
				position_unit = gripper_pos_max_s_;
			}
		} else {
			position_unit = int(convertRadToUnit(joint_name, position));

		}
		return position_unit;
	}

	// FIX - maybe break into separate reads since serial driver reads one at a time anyway
	// Read all joint positions
	void xarm::readJointPositions(std::map<std::string, int> &pos_map)
	{
		RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "readJointsPosition start");

		std::vector<uint16_t> pos(NUM_JOINTS, 0);
		int joint_id;
		if (drvr_->readJointPositionAll(pos)) {
			for (auto const &j: joint_name_map_) {
				std::string name = j.first;

				// If mirrored joint 1
				if (name == "xarm_1_joint_mirror") {
					joint_id = joint_name_map_["xarm_1_joint"];
				} else {
					joint_id = j.second;
				}

				// 1-based joint IDs
				uint16_t p = pos[joint_id - 1];
				pos_map[name] = p;

				RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Read servo %s, pos= %d, %f",
					name.c_str(), p, jointValueToPosition(name, p));
			}
		} else {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Failed to read joint position");
		}
		RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "readJointsPosition exit");
	}

	// Set the specified joint position
	void  xarm::setJointPosition(std::string joint_name, int position, int time)
	{
		RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Set servo %s, pos= %d, time %d",
				joint_name.c_str(), position, time);

		if (!drvr_->setJointPosition(joint_name_map_[joint_name], position, time)) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Failed to set joint position for servo %s",
				joint_name.c_str());
		}
		return;
	}

	// Check for the file that is used to manually nenable/disable this mode for testing
	bool xarm::manual_mode_enabled()
	{
		return access(MANUAL_MODE_ENABLE_FILE.c_str(), F_OK ) != -1;
	}

	// Manual mode turns off the motor in the servo so you can back drive to a desired position
	void xarm::set_manual_mode(bool enable)
	{
		RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Enable manual mode: %C", enable? 'Y': 'N');

		if (!drvr_->setManualModeAll(enable, NUM_JOINTS)) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Failed to set joint mode enable");
		}			
	}

	void xarm::Process()
	{
		int read_pos_delay_cnt = 0;
		int ck_for_manual_mode_cnt = 0;
		bool manual_mode = false;
		bool idle = false;
		int ck_for_idle_cnt = 0;

		while (run_) {
			auto next_update_time = std::chrono::steady_clock::now();

			RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Update");

			if (idle && --ck_for_manual_mode_cnt <= 0) {
				ck_for_manual_mode_cnt = UPDATE_CNT_CHK_FOR_MANUAL_MODE;
				bool enabled = manual_mode_enabled();
				if (manual_mode) {
					if (!enabled) {
						set_manual_mode(false);
						manual_mode = false;
					} else {
						// Periodically print each joint position while in manual mode
						RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "In Manual mode, joint positions:");
						for (auto const &p: last_pos_get_map_) {
							RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "  Pos: %d,  Joint: %s", p.second, p.first.c_str());
						}
					}
				} else if (!manual_mode && enabled) {
					set_manual_mode(true);
					manual_mode = true;
				}
			}

			bool new_cmd = false;
			std::map<std::string, int> cmd;
			{
				std::lock_guard<std::mutex> guard(mutex_);
				cmd = last_pos_set_map_;
				new_cmd = new_cmd_;
				new_cmd_ = false;
			}

			if (new_cmd) {
				read_pos_delay_cnt = 1;

				for (auto const &c: cmd) {
					int set_pos = c.second;
					const std::string &joint = c.first;
					RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Update, joint %s, pos= %d", joint.c_str(), set_pos);
					// For small changes such as when a trajectory is being followed (ie. by moveit2)the movement should
					// occur over the update duration of this loop while new updates to the trajectory are
					// being received.  For cases where the position represents a large change such as during initialization
					// when driven with arbitrary joint settings, use a slow movement time to keep from jerking the arm.
					setJointPosition(joint, set_pos, 
						abs(set_pos - last_pos_get_map_[joint]) < SMALL_CHANGE? UPDATE_PERIOD_MOVING_MS: LARGE_CHANGE_MOVE_TIME);
				}
				if (idle) {
					RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Entering running mode");
					idle = false;
				}
				ck_for_idle_cnt = 0;
				ck_for_manual_mode_cnt = 0;
			} else if (!idle && ck_for_idle_cnt++ > IDLE_ENTRY_CNT) {
				idle = true;
				RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Entering idle mode");
			}

			// Don't read while moving since it causes jerks in the motion.  Update after commands stop.
			if (!new_cmd && --read_pos_delay_cnt <= 0) {
				read_pos_delay_cnt = 5;
				std::map<std::string, int> pos_map = last_pos_get_map_;
				readJointPositions(pos_map);
				{
					std::lock_guard<std::mutex> guard(mutex_);
					last_pos_get_map_ = pos_map;
				}
			}

			next_update_time += std::chrono::milliseconds(idle? UPDATE_PERIOD_IDLE_MS: UPDATE_PERIOD_MOVING_MS);

			// Sleep for whatever remaining time until the next update
		    std::this_thread::sleep_until(next_update_time);
		}
	}

}
