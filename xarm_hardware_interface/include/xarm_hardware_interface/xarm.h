
#ifndef XARM__H
#define XARM__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <map>
#include <vector>
#include <thread>
#include <mutex>

#include "xarm_hardware_interface/xarm_drvr.hpp"

namespace xarm
{
	class xarm
	{
		struct JointRangeLimits {
			float range_rad;	// Range in radians corresponding to max-min servo units
			int min;			// Range min, servo units
			int max;			// Range max, servo units
			int mid;			// Range min, servo units
			int def;			// Default position, servo units
			int invert_factor;	// 1 or -1 to invert range
		};

		struct Position {
			int pos;			// Last position
			bool changed;		// True if position changed
		};

		using PositionMap = std::map<std::string, struct Position>;

		public:
			xarm();
			~xarm();

			bool init();

			void setAllJointPositions(const std::vector<double> &commands, const std::vector<std::string> &joints);
			void getAllJointPositions(std::vector<double> &positions, const std::vector<std::string> &joints);

			double readDefaultPosition(std::string joint_name);

		private:
			void Process();

			double jointValueToPosition(std::string joint_name, int jointValue);
			int positionToJointValue(std::string joint_name, double position);

			double convertUnitToRad(std::string joint_name, int unit);
			int convertRadToUnit(std::string joint_name, double rad);

			void readJointPositions(PositionMap &pos_map);
			void setJointPosition(std::string joint_name, int position, int time);

			void set_manual_mode(bool enable);
			bool manual_mode_enabled();

			bool inited_;
			bool run_;

			std::unique_ptr<xarm_drvr> drvr_;

			std::mutex mutex_;
			std::thread thread_;

			std::map<std::string, int> joint_name_map_;
			std::map<std::string, struct JointRangeLimits> joint_range_limits_;

			// Map of joint to the last position set on the joint (in xarm units)
			PositionMap last_pos_set_map_;
			// Map of joint to the last position status read for the joint (in xarm units)
			PositionMap last_pos_get_map_;

			double gripper_pos_min_m_;
			double gripper_pos_min_s_;
			double gripper_pos_max_s_;
			double gripper_pos_m_to_s_factor_;

			bool new_cmd_;
	};
}

#endif
