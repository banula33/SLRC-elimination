#include "MessageHandler.h"

MessageHandler::MessageHandler(MoveController &robotRef, CommProtocol &commRef)
    : robot(robotRef), comm(commRef) {}

void MessageHandler::processIncomingMessages()
{
  CommPacket packet;
  if (comm.readPacket(packet))
  {
    switch (packet.msgId)
    {
    case MSG_ID_DRIVE_MOTOR_CMD:
      handleDriveMotorCmd(packet);
      break;
    // Additional command cases can be handled here (ARM_CMD, etc)
    default:
      break;
    }
  }
}

void MessageHandler::handleDriveMotorCmd(const CommPacket &packet)
{
  DriveMotorCmdPayload cmd;
  if (decodePayload(packet, cmd))
  {
    // Direct translation of ROS2 v_left and v_right commands (mm/s)
    // to our MoveController
    robot.setMotorSpeedsMMS(cmd.left_speed_mm_s, cmd.right_speed_mm_s);
  }
}
