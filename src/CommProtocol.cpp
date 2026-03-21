#include "CommProtocol.h"

CommProtocol::CommProtocol(Stream &stream)
    : io(&stream), state(WAIT_SYNC), payloadIndex(0)
{
  memset(&packet, 0, sizeof(packet));
}

void CommProtocol::resetParser()
{
  state = WAIT_SYNC;
  payloadIndex = 0;
  packet.msgId = 0;
  packet.len = 0;
  packet.checksum = 0;
}

uint8_t CommProtocol::computeChecksum(uint8_t msgId, uint8_t len, const uint8_t *payload)
{
  uint8_t cs = COMM_SYNC ^ msgId ^ len;
  for (uint8_t i = 0; i < len; ++i)
  {
    cs ^= payload[i];
  }
  return cs;
}

bool CommProtocol::readPacket(CommPacket &outPacket)
{
  while (io->available() > 0)
  {
    uint8_t byteIn = static_cast<uint8_t>(io->read());

    switch (state)
    {
    case WAIT_SYNC:
      if (byteIn == COMM_SYNC)
      {
        state = READ_MSG_ID;
      }
      break;

    case READ_MSG_ID:
      packet.msgId = byteIn;
      state = READ_LEN;
      break;

    case READ_LEN:
      packet.len = byteIn;
      if (packet.len > COMM_MAX_PAYLOAD)
      {
        resetParser();
      }
      else if (packet.len == 0)
      {
        state = READ_CHECKSUM;
      }
      else
      {
        payloadIndex = 0;
        state = READ_PAYLOAD;
      }
      break;

    case READ_PAYLOAD:
      packet.payload[payloadIndex++] = byteIn;
      if (payloadIndex >= packet.len)
      {
        state = READ_CHECKSUM;
      }
      break;

    case READ_CHECKSUM:
    {
      packet.checksum = byteIn;
      uint8_t expected = computeChecksum(packet.msgId, packet.len, packet.payload);
      if (packet.checksum == expected)
      {
        outPacket = packet;
        resetParser();
        return true;
      }
      resetParser();
      break;
    }
    }
  }

  return false;
}

bool CommProtocol::sendPacket(uint8_t msgId, const uint8_t *payload, uint8_t len)
{
  if (len > COMM_MAX_PAYLOAD)
    return false;

  uint8_t header[3];
  header[0] = COMM_SYNC;
  header[1] = msgId;
  header[2] = len;
  uint8_t checksum = computeChecksum(msgId, len, payload);

  io->write(header, sizeof(header));
  if (len > 0)
  {
    io->write(payload, len);
  }
  io->write(checksum);
  return true;
}
