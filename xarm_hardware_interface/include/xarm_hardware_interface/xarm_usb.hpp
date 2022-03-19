
#ifndef XARM_USB__H
#define XARM_USB__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "xarm_drvr.hpp"

namespace xarm
{
	class xarm_usb: public xarm_drvr
	{
		public:
			xarm_usb();
			~xarm_usb();

			bool open(const std::string &portname) override;
			void close() override;

			bool readJointPositionAll(std::vector<uint16_t> &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			void printDeviceInformation();

			hid_device *handle_;
	};
}

#endif // XARM_USB__H
