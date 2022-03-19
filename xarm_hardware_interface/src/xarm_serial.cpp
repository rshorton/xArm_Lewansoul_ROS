// Serial servo bus control for LewanSoul/Hiwonder servos

#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <chrono>     

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "xarm_hardware_interface/xarm_serial.hpp"
#include "xarm_hardware_interface/serial_servo_bus.h"

///////////////////////////////////////////////////////////////////////////////////////////
// From https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c

static int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

static void set_mincount(int fd, int mcount, int to_x100ms)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        fprintf(stderr, "Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = to_x100ms;

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        fprintf(stderr, "Error tcsetattr: %s\n", strerror(errno));
}
///////////////////////////////////////////////////////////////////////////////////////////

namespace xarm
{
	xarm_serial::xarm_serial():
		xarm_drvr(),
		fd_(-1)
	{
	}

	xarm_serial::~xarm_serial()
	{
		close();
	}

	bool xarm_serial::open(const std::string &portname)
	{
		if (fd_ > 0) {
			return false;
		}

		fd_ = ::open(portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    	if (fd_ < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "xArm, unable to open serial port [%s]",
				strerror(errno));
			return false;
		}

  		// 115200, 8 bits, no parity, 1 stop bit
	    set_interface_attribs(fd_, B115200);
		set_mincount(fd_, 0, 5);  
		RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "xArm device opened ");
		return true;
	}

	void xarm_serial::close()
	{
		if (fd_ > 0) {
			::close(fd_);
			fd_ = -1;
		}
	}

	bool xarm_serial::readJointPositionAll(std::vector<uint16_t> &pos)
	{
		RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "readJointPositionAll");
		bool bOk = true;
		for (size_t i = 0; i < pos.size(); i++) {
			if (!LobotSerialServoReadPosition(fd_, i + 1, pos[i])) {
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Failed to read servo %lu position", i + 1);
				bOk = false;
			}
		}
		return bOk;
	}

	bool xarm_serial::setJointPosition(int id, uint16_t pos, uint16_t time)
	{
		if (!LobotSerialServoMove(fd_, id, pos, time)) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Failed to set servo %u position", id);
			return false;
		}
		return true;		
	}

	bool xarm_serial::setManualModeAll(bool enable, int count)
	{
		bool bOk = false;
		for (int i = 0; i < count; i++) {
			if (enable) {
				bOk = LobotSerialServoUnload(fd_, i + 1);
			} else {
				bOk = LobotSerialServoLoad(fd_, i + 1);
			}
			if (!bOk) {
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Failed to set enable mode on servo %d", i + 1);
				bOk = false;
			}
		}
		return bOk;
	}
}
