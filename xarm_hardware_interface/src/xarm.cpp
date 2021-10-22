#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "xarm_hardware_interface/xarm.h"

#define MAX_STR 255
#define PI 3.14159265359

using std::string;

namespace xarm
{
	xarm::xarm():
		inited(false)
	{
	}

	xarm::~xarm()
	{
		if (inited) {
			hid_close(handle);

			/* Free static HIDAPI objects. */
			hid_exit();
		}
	}

	bool xarm::init()
	{
		if (inited) {
			return false;
		}

		// Initialize the hidapi library
		if (hid_init()) {
			return false;
		}

		int found=0;
		printDeviceInformation();
		devs = hid_enumerate(0x0, 0x0);
		cur_dev = devs;
		while (cur_dev) {

			std::wstring ws(cur_dev->product_string);
			string product(ws.begin(), ws.end());

			if (product=="LOBOT")
			{
				RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "LOBOT found ");
				found=1;
				break;
			}
			cur_dev = cur_dev->next;
		}
		if (found==0)
		{
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "LOBOT not found, make sure it is power on ");
			return false;
		}

		handle = hid_open_path(cur_dev->path);

		if (!handle) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "unable to open device");
			return false;
		}
		RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Device opened ");
		hid_free_enumeration(devs);

		//Dictionary of joint_names to joint_id
		joint_name_map.insert(std::make_pair("xarm_1_joint" , 1));
		joint_name_map.insert(std::make_pair("xarm_1_joint_mirror" , 11));
		joint_name_map.insert(std::make_pair("xarm_2_joint" , 2));
		joint_name_map.insert(std::make_pair("xarm_3_joint" , 3));
  		joint_name_map.insert(std::make_pair("xarm_4_joint" , 4));
		joint_name_map.insert(std::make_pair("xarm_5_joint" , 5));
		joint_name_map.insert(std::make_pair("xarm_6_joint" , 6));


		// gripper range servo units:             700   -  200
		// corresponding to phy units of meters:  0.003 - 0.028
		// 0.3 is 1/2 the total grip width since mimic joint used in urdf
		gripper_pos_min_m = 0.003;  // meters
		gripper_pos_min_s = 700.0; // servo units
		gripper_pos_max_s = 200.0;
								  // scale factor: mult scale by phy units in meter to get servo units
		gripper_pos_m_to_s_factor = (gripper_pos_max_s - gripper_pos_min_s)/(0.028 - gripper_pos_min_m);

		matrix_unit_transform["xarm_2_joint"][0][0] = 200;	// min in servo units
		matrix_unit_transform["xarm_2_joint"][0][1] = 980;	// max in servo units
		matrix_unit_transform["xarm_2_joint"][0][2] = 1;	// -1 to invert range
		matrix_unit_transform["xarm_3_joint"][0][0] = 140;
		matrix_unit_transform["xarm_3_joint"][0][1] = 880;
		matrix_unit_transform["xarm_3_joint"][0][2] = -1;
		matrix_unit_transform["xarm_4_joint"][0][0] = 870;
		matrix_unit_transform["xarm_4_joint"][0][1] = 130;
		matrix_unit_transform["xarm_4_joint"][0][2] = -1;
		matrix_unit_transform["xarm_5_joint"][0][0] = 140;
		matrix_unit_transform["xarm_5_joint"][0][1] = 880;
		matrix_unit_transform["xarm_5_joint"][0][2] = -1;
		matrix_unit_transform["xarm_6_joint"][0][0] = 90;
		matrix_unit_transform["xarm_6_joint"][0][1] = 845;
		matrix_unit_transform["xarm_6_joint"][0][2] = 1;

		inited = true;
		return true;
	}

	void xarm::printDeviceInformation()
	{
		devs = hid_enumerate(0x0, 0x0);
		cur_dev = devs;
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
		double range = matrix_unit_transform[joint_name][0][1] - matrix_unit_transform[joint_name][0][0];
		// Mid-range in servo units
		//double b = matrix_unit_transform[joint_name][0][1] - range/2;
		double b = 500.0;
		return (range*rad/PI*matrix_unit_transform[joint_name][0][2]) + b;
	}

	double xarm::convertUnitToRad(std::string joint_name, int unit)
	{
		// Range in servo units
		double range = matrix_unit_transform[joint_name][0][1] - matrix_unit_transform[joint_name][0][0];
		// Mid-range in servo units
		double b = 500.0;
		//double b = matrix_unit_transform[joint_name][0][1] - range/2;
		return (unit - b)*PI*matrix_unit_transform[joint_name][0][2]/range;
	}
	std::vector<double> xarm::readJointsPosition(std::vector<std::string> joint_names)
	{
		int res;
		std::vector<double> joint_positions;
		unsigned char buf[65];
		//RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "readJointsPosition start");

		joint_positions.resize(joint_names.size());
		buf[0] = 0x55;
		buf[1] = 0x55;
		buf[2] = 9;
		buf[3] = 21;
		buf[4] = 6;
		buf[5] = 1;
		buf[6] = 2;
		buf[7] = 3;
		buf[8] = 4;
		buf[9] = 5;
		buf[10] = 6;
		res = hid_write(handle, buf, 17);

		if (res < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to write()");
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Error: %ls", hid_error(handle));
		}

		res = 0;
		while (res == 0) {
			res = hid_read(handle, buf, sizeof(buf));
			if (res > 0) {
				break;
			}
			if (res == 0)
				RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "waiting...");
			if (res < 0)
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to read()");
			usleep(500*1000);
		}

		int p_lsb, p_msb, unit, joint_id;
		for (int i=0; i<(int)joint_names.size(); i++){
			// If mirrored joint 1
			bool bJ1Mirror = joint_names[i] == "xarm_1_joint_mirror";
			if (bJ1Mirror) {
				joint_id = joint_name_map["xarm_1_joint"];
			} else {
				joint_id = joint_name_map[joint_names[i]];
			}
			//int id = buf[2+3*joint_id];
			p_lsb= buf[2+3*joint_id+1];
			p_msb= buf[2+3*joint_id+2];
			unit= (p_msb << 8) + p_lsb;

			if (joint_id == 1 || joint_id == 11) {
				float pos = (float)unit;
				if (pos > gripper_pos_min_s) {
					pos = gripper_pos_min_s;
				} else if (pos < gripper_pos_max_s) {
					pos = gripper_pos_max_s;
				}
				joint_positions[i] = (pos - gripper_pos_min_s)/gripper_pos_m_to_s_factor + gripper_pos_min_m;
				if (i == 1) {
					joint_positions[i] *= -1;
				}
				//RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "servo %s (%d), servo units %d, pos %f",
				//	joint_names[i].c_str(), id, unit, joint_positions[i]);
			} else {
				joint_positions[i] = convertUnitToRad(joint_names[i], unit);
				//RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "servo %s (%d), servo units %d, pos %f rad",
				//	 joint_names[i].c_str(), id, unit,joint_positions[i]);
			}
		}
		//RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "readJointsPosition exit ");

		return joint_positions;
	}

	void  xarm::setJointPosition(std::string joint_name, double position, int time=1000)
	{
		unsigned char buf[65];
		unsigned char t_lsb,t_msb, p_lsb, p_msb;
		int res;
		int position_unit = 0;

		if (joint_name == "xarm_1_joint") {
			double pos_in = position;
			if (pos_in < gripper_pos_min_m) {
				pos_in = gripper_pos_min_m;
			}
			position_unit = (int)((pos_in - gripper_pos_min_m)*gripper_pos_m_to_s_factor + gripper_pos_min_s);
			//RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Set servo %s, pos_in= %f, unit %d ", pos_in,

			if (position_unit < gripper_pos_max_s) {
				position_unit = gripper_pos_max_s;
			}
			RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "Set servo %s, pos= %f, position_unit %d ",
					joint_name.c_str(), position, position_unit);
		} else if (joint_name == "xarm_1_joint_mirror") {
			return;
		} else {
			position_unit = int(convertRadToUnit(joint_name, position));
		}

        t_lsb= time & 0xFF;
		t_msb = time >> 8;
		p_lsb = position_unit & 0xFF;
		p_msb = position_unit >> 8;

		buf[0] = 0x55;
		buf[1] = 0x55;
		buf[2] = 8;
		buf[3] = 0x03;
		buf[4] = 1;
		buf[5] = t_lsb;
		buf[6] = t_msb;
		buf[7] = joint_name_map[joint_name];
		buf[8] = p_lsb;
		buf[9] = p_msb;

		res = hid_write(handle, buf, 17);

		if (res < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to write()");
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Error: %ls", hid_error(handle));
		}

	}

}
