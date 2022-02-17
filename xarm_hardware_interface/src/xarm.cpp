#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>     

#include "rclcpp/rclcpp.hpp"
#include "xarm_hardware_interface/xarm.h"

#define MAX_STR 255
#define PI 3.14159265359
#define INVALID_POS 99999	// Invalid servo value

const int UPDATE_PERIOD_MS = 30;

// How often to check for the file that indicates the control loop should be
// in manual mode where the user can manually move the robot arm (for specifying
// positions for training.)
const int UPDATE_CNT_CHK_FOR_MANUAL_MODE = (2000/UPDATE_PERIOD_MS);
// File to create to enable the manual mode
const std::string MANUAL_MODE_ENABLE_FILE = "/tmp/xarm_enable_manual_mode";

// See comments below
const int SMALL_CHANGE = 20;
const int LARGE_CHANGE_MOVE_TIME = 1500;

namespace xarm
{
	xarm::xarm():
		inited_(false),
		run_(false),
		handle_(NULL),
		devs_(NULL),
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

			hid_close(handle_);
			/* Free static HIDAPI objects. */
			hid_exit();
		}
	}

	bool xarm::init()
	{
		if (inited_) {
			return false;
		}

		// Initialize the hidapi library
		if (hid_init()) {
			return false;
		}

		bool found = false;
		printDeviceInformation();
		devs_ = hid_enumerate(0x0, 0x0);
		struct hid_device_info* cur_dev = devs_;

		while (cur_dev) {

			std::wstring ws(cur_dev->product_string);
			std::string product(ws.begin(), ws.end());

			if (product=="LOBOT") {
				RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "LOBOT found ");
				found = true;
				break;
			}
			cur_dev = cur_dev->next;
		}

		if (!found) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "LOBOT not found, make sure it is power on ");
			return false;
		}

		handle_ = hid_open_path(cur_dev->path);

		if (!handle_) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "unable to open device");
			return false;
		}
		RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Device opened ");
		hid_free_enumeration(devs_);

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
		gripper_pos_min_s_ = 700.0; // servo units
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

	void xarm::printDeviceInformation()
	{
		devs_ = hid_enumerate(0x0, 0x0);
		struct hid_device_info *cur_dev = devs_;
		while (cur_dev) {
#if 0
			printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
			printf("\n");
			printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
			printf("  Product:      %ls\n", cur_dev->product_string);
			printf("  Release:      %hx\n", cur_dev->release_number);
			printf("  Interface:    %d\n",  cur_dev->interface_number);
			printf("\n");
#endif
			cur_dev = cur_dev->next;
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

	// Read all joint positions
	void xarm::readJointPositions(std::map<std::string, int> &pos_map)
	{
		//RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "readJointsPosition start");

		unsigned char buf[65];
		buf[0] = 0x55;
		buf[1] = 0x55;
		buf[2] = 9;
		buf[3] = 21;
		buf[4] = 7;
		buf[5] = 1;
		buf[6] = 2;
		buf[7] = 3;
		buf[8] = 4;
		buf[9] = 5;
		buf[10] = 6;
		buf[11] = 7;
		int res = hid_write(handle_, buf, 17);

		if (res < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to write()");
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Error: %ls", hid_error(handle_));
		}

		do {
			res = hid_read(handle_, buf, sizeof(buf));
			if (res > 0) {
				break;
			} else if (res == 0) {
				RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "waiting...");
			} else if (res < 0) {
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to read()");
			}				
			usleep(500*1000);
		} while (res == 0);

		int p_lsb, p_msb, unit, joint_id;

		for (auto const &j: joint_name_map_) {
			std::string name = j.first;

			// If mirrored joint 1
			if (name == "xarm_1_joint_mirror") {
				joint_id = joint_name_map_["xarm_1_joint"];
			} else {
				joint_id = j.second;
			}

			p_lsb= buf[2+3*joint_id+1];
			p_msb= buf[2+3*joint_id+2];
			unit= (p_msb << 8) + p_lsb;

			pos_map[name] = unit;

			RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Read servo %s, pos= %d, %f",
				name.c_str(), unit, jointValueToPosition(name, unit));
		}
		//RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "readJointsPosition exit ");
	}

	// Set the specified joint position
	void  xarm::setJointPosition(std::string joint_name, int position, int time)
	{
		unsigned char buf[65];
		unsigned char t_lsb,t_msb, p_lsb, p_msb;
		int res;

		RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Set servo %s, pos= %d, time %d",
				joint_name.c_str(), position, time);

        t_lsb= time & 0xFF;
		t_msb = time >> 8;
		p_lsb = position & 0xFF;
		p_msb = position >> 8;

		buf[0] = 0x55;
		buf[1] = 0x55;
		buf[2] = 8;
		buf[3] = 0x03;
		buf[4] = 1;
		buf[5] = t_lsb;
		buf[6] = t_msb;
		buf[7] = joint_name_map_[joint_name];
		buf[8] = p_lsb;
		buf[9] = p_msb;

		res = hid_write(handle_, buf, 17);

		if (res < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to write()");
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Error: %ls", hid_error(handle_));
		}
	}

	bool xarm::manual_mode_enabled()
	{
		return access(MANUAL_MODE_ENABLE_FILE.c_str(), F_OK ) != -1;
	}

	void xarm::set_manual_mode(bool enable)
	{
		if (enable) {
			// Send the 'off' command for all servos
			unsigned char buf[65];
			buf[0] = 0x55;
			buf[1] = 0x55;
			buf[2] = 9;
			buf[3] = 20;
			buf[4] = 6;
			buf[5] = 1;
			buf[6] = 2;
			buf[7] = 3;
			buf[8] = 4;
			buf[9] = 5;
			buf[10] = 6;
			int res = hid_write(handle_, buf, 17);

			if (res < 0) {
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to write 'servo off' command");
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Error: %ls", hid_error(handle_));
			}
		} else {
			// Send a position command to each servo to re-enable it
			for (auto const &p: last_pos_get_map_) {
				if (p.first != "xarm_1_joint_mirror") {
					setJointPosition(p.first, p.second, 1000);
				}					
			}
		}
		RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Enabled manual mode: %C", enable? 'Y': 'N');
	}

	void xarm::Process()
	{
		int read_pos_delay_cnt = 0;
		int ck_for_manual_mode_cnt = 0;
		bool manual_mode = false;

		while (run_) {
			auto next_update_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(UPDATE_PERIOD_MS);

			RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Update");

			if (--ck_for_manual_mode_cnt <= 0) {
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
				read_pos_delay_cnt = 1;
			}

			if (new_cmd) {
				for (auto const &c: cmd) {
					int set_pos = c.second;
					const std::string &joint = c.first;
					RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "Update, joint %s, pos= %d", joint.c_str(), set_pos);
					// For small changes such as when a trajectory is being followed (ie. by moveit2)the movement should
					// occur over the update duration of this loop while new updates to the trajectory are
					// being received.  For cases where the position represents a large change such as during initialization
					// when driven with arbitrary joint settings, use a slow movement time to keep from jerking the arm.
					setJointPosition(joint, set_pos, 
						abs(set_pos - last_pos_get_map_[joint]) < SMALL_CHANGE? UPDATE_PERIOD_MS: LARGE_CHANGE_MOVE_TIME);
				}
			}

			// Don't read while moving since it causes jerks in the motion.  Update after commands stop.
			if (!new_cmd && --read_pos_delay_cnt <= 0) {
				std::map<std::string, int> pos_map = last_pos_get_map_;
				readJointPositions(pos_map);
				{
					std::lock_guard<std::mutex> guard(mutex_);
					last_pos_get_map_ = pos_map;
				}
			}

			// Sleep for whatever remaining time until the next update
		    std::this_thread::sleep_until(next_update_time);
		}
	}

}
