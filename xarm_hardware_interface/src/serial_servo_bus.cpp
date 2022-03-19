// Serial servo bus control for LewanSoul/Hiwonder servos
//
// Based on Arduino source code provided by HiWonder
//

#include <iostream>
#include <cstdint>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "xarm_hardware_interface/serial_servo_bus.h"

#define GET_LOW_uint8_t(A) (uint8_t)((A))
// Macro function  get lower 8 bits of A
#define GET_HIGH_uint8_t(A) (uint8_t)((A) >> 8)
// Macro function  get higher 8 bits of A
#define uint8_t_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
// put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits
// integer

#define LOBOT_SERVO_FRAME_HEADER 0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE 1
#define LOBOT_SERVO_MOVE_TIME_READ 2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ 8
#define LOBOT_SERVO_MOVE_START 11
#define LOBOT_SERVO_MOVE_STOP 12
#define LOBOT_SERVO_ID_WRITE 13
#define LOBOT_SERVO_ID_READ 14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST 17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE 18
#define LOBOT_SERVO_ANGLE_OFFSET_READ 19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE 20
#define LOBOT_SERVO_ANGLE_LIMIT_READ 21
#define LOBOT_SERVO_VIN_LIMIT_WRITE 22
#define LOBOT_SERVO_VIN_LIMIT_READ 23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ 25
#define LOBOT_SERVO_TEMP_READ 26
#define LOBOT_SERVO_VIN_READ 27
#define LOBOT_SERVO_POS_READ 28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE 29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ 30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ 32
#define LOBOT_SERVO_LED_CTRL_WRITE 33
#define LOBOT_SERVO_LED_CTRL_READ 34
#define LOBOT_SERVO_LED_ERROR_WRITE 35
#define LOBOT_SERVO_LED_ERROR_READ 36

static uint8_t LobotCheckSum(uint8_t buf[]) {
  uint8_t i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (uint8_t)temp;
  return i;
}

static bool writeMsg(int fd, const uint8_t *buf, int len) {
  if (len != write(fd, buf, len)) {
    return false;
  }
  tcdrain(fd);
  return true;
}

bool LobotSerialServoMove(int fd, uint8_t id, int16_t position, uint16_t time) {
  const int msgLen = 10;
  uint8_t buf[msgLen];
  if (position < 0) {
    position = 0;
  }
  if (position > 1000) {
    position = 1000;
  }
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = msgLen - 3;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_uint8_t(position);
  buf[6] = GET_HIGH_uint8_t(position);
  buf[7] = GET_LOW_uint8_t(time);
  buf[8] = GET_HIGH_uint8_t(time);
  buf[9] = LobotCheckSum(buf);
  return writeMsg(fd, buf, msgLen);
}

bool LobotSerialServoStopMove(int fd, uint8_t id) {
  const int msgLen = 6;
  uint8_t buf[msgLen];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = msgLen - 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  return writeMsg(fd, buf, msgLen);
}

bool LobotSerialServoSetID(int fd, uint8_t oldID, uint8_t newID) {
  const int msgLen = 7;
  uint8_t buf[msgLen];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = msgLen - 3;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  return writeMsg(fd, buf, msgLen);
}

bool LobotSerialServoSetMode(int fd, uint8_t id, uint8_t Mode, int16_t Speed) {
  const int msgLen = 10;
  uint8_t buf[msgLen];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = msgLen - 3;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_uint8_t((uint16_t)Speed);
  buf[8] = GET_HIGH_uint8_t((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);
  return writeMsg(fd, buf, msgLen);
}

// Doesn't work
bool LobotSerialServoLoad(int fd, uint8_t id) {
  const int msgLen = 7;
  uint8_t buf[msgLen];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = msgLen - 3;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  //std::cerr << "load" << std::endl;  
  return writeMsg(fd, buf, msgLen);
}

// Doesn't work
bool LobotSerialServoUnload(int fd, uint8_t id) {
  const int msgLen = 7;
  uint8_t buf[msgLen];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = msgLen - 3;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);

  //std::cerr << "unload" << std::endl;
  return writeMsg(fd, buf, msgLen);
}

static bool ReceiveResponse(int fd, uint8_t *resp, int max_resp) {
  bool frameStarted = false;
  uint8_t frameCount = 0;
  uint8_t dataCount = 0;
  uint8_t dataLength = 2;
  uint8_t rxBuf;
  uint8_t recvBuf[32];
  int ret;

  while (1) {
    if ((ret = read(fd, &rxBuf, 1)) > 0) {
      usleep(100);
      if (!frameStarted) {
        if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
          frameCount++;
          if (frameCount == 2) {
            frameCount = 0;
            frameStarted = true;
            dataCount = 1;
          }
        } else {
          frameStarted = false;
          dataCount = 0;
          frameCount = 0;
        }
      }
      if (frameStarted) {
        recvBuf[dataCount] = (uint8_t)rxBuf;
        if (dataCount == 3) {
          dataLength = recvBuf[dataCount];
          if (dataLength < 3 || dataCount > 7) {
            dataLength = 2;
            frameStarted = false;
          }
        }
        dataCount++;
        if (dataCount == dataLength + 3) {

          if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
            frameStarted = false;
            if (dataLength <= max_resp) {
              memcpy(resp, recvBuf + 4, dataLength);
              return true;
            }
          }
          return false;
        }
      }
    } else if (ret < 0) {
      return false;
    } else if (ret == 0) {
      // timeout
    }
  }
}

static bool ReadCommand2ByteReturn(int fd, uint8_t id, uint8_t cmd,
                                   uint16_t &val) {
  val = -2048;
  const int msgLen = 6;
  uint8_t buf[msgLen];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = msgLen - 3;
  buf[4] = cmd;
  buf[5] = LobotCheckSum(buf);

  if (writeMsg(fd, buf, msgLen)) {
    if (ReceiveResponse(fd, buf, msgLen)) {
      val = (int16_t)uint8_t_TO_HW(buf[2], buf[1]);
      return true;
    }
  }
  return false;
}

bool LobotSerialServoReadPosition(int fd, uint8_t id, uint16_t &position) {
  return ReadCommand2ByteReturn(fd, id, LOBOT_SERVO_POS_READ, position);
}

bool LobotSerialServoReadVin(int fd, uint8_t id, uint16_t &vin) {
  return ReadCommand2ByteReturn(fd, id, LOBOT_SERVO_VIN_READ, vin);
}
