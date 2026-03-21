#pragma once

#include <Arduino.h>
#include <string.h>

static const uint8_t COMM_SYNC = 0xAA;
static const uint8_t COMM_MAX_PAYLOAD = 64;

enum MsgId : uint8_t
{
  MSG_ID_IR_ARRAY = 0x10,
  MSG_ID_ENCODER = 0x11,
  MSG_ID_ARM_CMD = 0x41,
  MSG_ID_GYRO = 0x12,
  MSG_ID_GYRO_MEASUREMENTS = 0x13,
  MSG_ID_DRIVE_CMD = 0x20,
  MSG_ID_DRIVE_MOTOR_CMD = 0x40,
  MSG_ID_STATUS = 0xF0,
  MSG_ID_HEARTBEAT = 0xF1
};

struct CommPacket
{
  uint8_t msgId;
  uint8_t len;
  uint8_t payload[COMM_MAX_PAYLOAD];
  uint8_t checksum;
};

struct __attribute__((packed)) DriveMotorCmdPayload
{
  int16_t left_speed_mm_s;
  int16_t right_speed_mm_s;
};

struct __attribute__((packed)) ArmCmdPayload
{
  uint8_t joint_id;
  int16_t angle_deg;
  uint8_t speed;
};

struct __attribute__((packed)) IrArrayPayload
{
  uint8_t values[8];
};

struct __attribute__((packed)) EncoderPayload
{
  int16_t left_cm;
  int16_t right_cm;
};

struct __attribute__((packed)) GyroPayload
{
  int16_t yaw_deg_x100;
  int16_t pitch_deg_x100;
};

struct __attribute__((packed)) GyroMeasurementPayload
{
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  int16_t temperatureRaw;
};

struct __attribute__((packed)) SystemStatusPayload
{
  uint8_t battery_mv;
  int8_t temperature_c;
};

class CommProtocol
{
public:
  explicit CommProtocol(Stream &stream);

  bool readPacket(CommPacket &outPacket);
  bool sendPacket(uint8_t msgId, const uint8_t *payload, uint8_t len);

  static uint8_t computeChecksum(uint8_t msgId, uint8_t len, const uint8_t *payload);

private:
  enum ParseState
  {
    WAIT_SYNC,
    READ_MSG_ID,
    READ_LEN,
    READ_PAYLOAD,
    READ_CHECKSUM
  };

  Stream *io;
  ParseState state;
  CommPacket packet;
  uint8_t payloadIndex;

  void resetParser();
};

template <typename T>
bool decodePayload(const CommPacket &packet, T &out)
{
  if (packet.len != sizeof(T))
    return false;
  memcpy(&out, packet.payload, sizeof(T));
  return true;
}

template <typename T>
void encodePayload(const T &payload, uint8_t *out)
{
  memcpy(out, &payload, sizeof(T));
}
