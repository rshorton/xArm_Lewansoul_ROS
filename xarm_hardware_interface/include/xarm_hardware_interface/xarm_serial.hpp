
#ifndef XARM_SERIAL__H
#define XARM_SERIAL__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <vector>

#include "xarm_drvr.hpp"

namespace xarm
{
	class xarm_serial: public xarm_drvr
	{
		public:
			xarm_serial();
			~xarm_serial();

			bool open(const std::string &portname) override;
			void close() override;

			bool getJointPosition(int id, uint16_t &pos) override;
			bool setJointPosition(int id, uint16_t pos, uint16_t time) override;
			bool setManualModeAll(bool enable, int count) override;

		private:
			int fd_;
	};
}

#endif // XARM_SERIAL__H
