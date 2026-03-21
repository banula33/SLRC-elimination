#pragma once

#include "CommProtocol.h"
#include "MoveController.h"

// MessageHandler is responsible for routing incoming network packets
// to the appropriate subsystems (like the MoveController).
class MessageHandler
{
private:
  MoveController &robot;
  CommProtocol &comm;

  void handleDriveMotorCmd(const CommPacket &packet);

public:
  MessageHandler(MoveController &robotRef, CommProtocol &commRef);

  // Call this inside loop() to process pending data
  void processIncomingMessages();
};
