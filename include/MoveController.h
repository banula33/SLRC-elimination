#pragma once
#include <Arduino.h>

class MoveController
{
private:
  int ENA, IN1, IN2, ENB, IN3, IN4;
  int rightEncA, rightEncB, leftEncA, leftEncB;
  bool encoderConfigured;
  float wheelDiameter;
  int encPPR;
  float wheelBase;
  int maxSpeed, minSpeed;
  float Kp, Ki, Kd;

public:
  // Pulse counters — updated by ISR, read from anywhere.
  // Must be volatile because ISRs modify them asynchronously.
  volatile long rightPulse = 0;
  volatile long leftPulse  = 0;

  MoveController(int ena, int in1, int in2, int enb, int in3, int in4,
                 float wheelDiameterCm, int encoderPPR, float wheelBaseCm,
                 int maxSpeed_ = 255, int minSpeed_ = 50);

  void begin();

  // Attach hardware interrupts on encoder channel-A pins.
  // Must be called once from setup() after begin().
  // rightA / leftA must be interrupt-capable pins (Mega: 2,3,18,19,20,21).
  void configureEncoders(int rightA, int rightB, int leftA, int leftB);

  // Public accessors for ISR to read channel-B pins
  int rightEncBPin() const { return rightEncB; }
  int leftEncBPin()  const { return leftEncB; }

  // Reset both pulse counters to zero.
  void resetEncoders();

  float pulsesToDistanceCm(long pulses) const;
  float getLeftDistanceCm()  const;
  float getRightDistanceCm() const;

  void setPID(float Kp_, float Ki_, float Kd_);
  void moveForwardCm(int cm);
  void moveBackwardCm(int cm);

  // In-place pivot turns using encoder feedback + PID.
  // Left wheel and right wheel spin in opposite directions.
  void turnRightDeg(int degrees);
  void turnLeftDeg(int degrees);

  void stop();
  void setMotorSpeedsPWM(int leftPWM, int rightPWM);
  void setMotorSpeedsMMS(int leftMMs, int rightMMs);
};