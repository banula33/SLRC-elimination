#pragma once
#include <Arduino.h>

class MoveController
{
private:
  int ENA, IN1, IN2, ENB, IN3, IN4;
  int rightEncA, rightEncB, leftEncA, leftEncB;
  int lastRightA, lastLeftA;
  bool encoderConfigured;
  float wheelDiameter;
  int encPPR;
  float wheelBase;
  int maxSpeed, minSpeed;
  float Kp, Ki, Kd;

  void updateEncoderCounts();

public:
  volatile long rightPulse = 0;
  volatile long leftPulse = 0;

  MoveController(int ena, int in1, int in2, int enb, int in3, int in4,
                 float wheelDiameterCm, int encoderPPR, float wheelBaseCm,
                 int maxSpeed_ = 255, int minSpeed_ = 50);

  void begin();
  void configureEncoders(int rightA, int rightB, int leftA, int leftB);
  void pollEncoders();
  float pulsesToDistanceCm(long pulses) const;
  float getLeftDistanceCm() const;
  float getRightDistanceCm() const;
  void setPID(float Kp_, float Ki_, float Kd_);
  void moveForwardCm(int cm);
  void moveBackwardCm(int cm);
  void stop();
  void setMotorSpeedsPWM(int leftPWM, int rightPWM);
  void setMotorSpeedsMMS(int leftMMs, int rightMMs);
};