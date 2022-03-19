
#ifndef SERIAL_SERVO_BUS__H
#define SERIAL_SERVO_BUS__H

#include <cstdint>

bool LobotSerialServoMove(int fd, uint8_t id, int16_t position, uint16_t time);
bool LobotSerialServoStopMove(int fd, uint8_t id);
bool LobotSerialServoSetID(int fd, uint8_t oldID, uint8_t newID);
bool LobotSerialServoSetMode(int fd, uint8_t id, uint8_t Mode, int16_t Speed);
bool LobotSerialServoLoad(int fd, uint8_t id);
bool LobotSerialServoUnload(int fd, uint8_t id);

bool LobotSerialServoReadPosition(int fd, uint8_t id, uint16_t &position);
bool LobotSerialServoReadVin(int fd, uint8_t id, uint16_t &vin);

#endif // SERIAL_SERVO_BUS__H
