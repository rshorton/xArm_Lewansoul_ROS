
#ifndef XARM__H
#define XARM__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <map>
#include <vector>

namespace xarm
{
	class xarm
	{
		public:
			xarm();
			~xarm();

			bool init();
			std::vector<double> readJointsPosition(std::vector<std::string> joint_names);
			void  setJointPosition(std::string joint_name, double position_rad, int time);
			double convertUnitToRad(std::string joint_name, int unit);
			int convertRadToUnit(std::string joint_name, double rad);

		private:
			hid_device *handle;
			struct hid_device_info *devs, *cur_dev;
			void printDeviceInformation();
			std::map<std::string, int> joint_name_map;
			std::map<std::string, int[1][2]> matrix_unit_transform;
			double gripper_pos_min_m;
			double gripper_pos_min_s;
			double gripper_pos_max_s;
			double gripper_pos_m_to_s_factor;

			bool inited;
	};
}

#endif
