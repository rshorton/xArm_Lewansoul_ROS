#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>     

#include "rclcpp/rclcpp.hpp"
#include "xarm_hardware_interface/xarm_usb.hpp"

///// WARNING, WARNING - this re-implemenation has not been tested.  My USB board died
///// so I switched to the serial board. 

namespace xarm
{
	xarm_usb::xarm_usb():
		xarm_drvr(),
		handle_(NULL)
	{
	}

	xarm_usb::~xarm_usb()
	{
		close();
	}

	bool xarm_usb::open(const std::string &portname)
	{
		(void)portname;
		if (handle_) {
			return false;
		}

		// Initialize the hidapi library
		if (hid_init()) {
			return false;
		}

		printDeviceInformation();
		struct hid_device_info *devs = hid_enumerate(0x0, 0x0);
		struct hid_device_info *cur_dev = devs;

		bool found = false;
		while (cur_dev) {
			std::wstring ws(cur_dev->product_string);
			std::string product(ws.begin(), ws.end());

			if (product == "LOBOT") {
				RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "xArm found ");
				found = true;
				break;
			}
			cur_dev = cur_dev->next;
		}

		if (found) {
			handle_ = hid_open_path(cur_dev->path);

			if (!handle_) {
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "xArm, unable to open device");
			} else {
				RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "xArm device opened ");
			}
		} else {		
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "xArm not found, make sure it is power on ");
			return false;
		}
		hid_free_enumeration(devs);
		return handle_? true: false;
	}

	void xarm_usb::close()
	{
		if (handle_) {
			hid_close(handle_);
			/* Frees static HIDAPI objects. */
			hid_exit();
			handle_ = NULL;
		}
	}

	void xarm_usb::printDeviceInformation()
	{
		struct hid_device_info *devs = hid_enumerate(0x0, 0x0);
		struct hid_device_info *cur_dev = devs;
		while (cur_dev) {
			printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
			printf("\n");
			printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
			printf("  Product:      %ls\n", cur_dev->product_string);
			printf("  Release:      %hx\n", cur_dev->release_number);
			printf("  Interface:    %d\n",  cur_dev->interface_number);
			printf("\n");
			cur_dev = cur_dev->next;
		}
		hid_free_enumeration(devs);
	}

	bool xarm_usb::readJointPositionAll(std::vector<uint16_t> &pos)
	{
		RCLCPP_DEBUG(rclcpp::get_logger("XArmSystemHardware"), "readJointPositionAll");

		int count = pos.size();
		{
			std::vector<uint8_t> buf {0x55, 0x55, 0, 21, 0};
			buf[2] = count + 2;
			buf[4] = count;

			for (int i = 0; i < count; i++) {
				buf.push_back(i + 1);
			}

			int res = hid_write(handle_, buf.data(), buf.size());
			if (res < 0) {
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to write()");
				RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Error: %ls", hid_error(handle_));
				return false;
			}
		}

		{
			std::vector<uint8_t> buf_out(4 + count*3, 0);
			int res = 0;
			do {
				res = hid_read(handle_, buf_out.data(), buf_out.size());
				if (res > 0) {
					break;
				} else if (res == 0) {
					RCLCPP_INFO(rclcpp::get_logger("XArmSystemHardware"), "waiting...");
				} else if (res < 0) {
					RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to read()");
					return false;
				}				
				usleep(500*1000);
			} while (res == 0);

			uint16_t p_lsb, p_msb;
			for (int i = 0; i < count; i++) {
				p_lsb= buf_out[2 + 3*(i + 1) + 1];
				p_msb= buf_out[2 + 3*(i + 1) + 2];
				pos[i] = (p_msb << 8) | p_lsb;
			}
		}			
		return true;
	}

	bool xarm_usb::setJointPosition(int id, uint16_t pos, uint16_t time)
	{
		std::vector<uint8_t> buf {0x55, 0x55, 8, 3, 1, 
									(uint8_t)(time & 0xFF),
									(uint8_t)(time >> 8),
									(uint8_t)id,
									(uint8_t)(pos & 0xFF),
									(uint8_t)(pos >> 8)};

		int res = hid_write(handle_, buf.data(), buf.size());
		if (res < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to write()");
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Error: %ls", hid_error(handle_));
			return false;
		}
		return true;
	}

	// Always enables manual mode for USB implemenation - send position command to exit manual mode.
	bool xarm_usb::setManualModeAll(bool enable, int count)
	{
		(void)enable;
		std::vector<uint8_t> buf {0x55, 0x55, 0, 20, 0};
		buf[2] = count + 2;
		buf[4] = count;

		for (int i = 0; i < count; i++) {
			buf.push_back(i + 1);
		}

		int res = hid_write(handle_, buf.data(), buf.size());
		if (res < 0) {
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Unable to write servo disable cmd");
			RCLCPP_ERROR(rclcpp::get_logger("XArmSystemHardware"), "Error: %ls", hid_error(handle_));
			return false;
		}
		return true;
	}
}
