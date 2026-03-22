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

  // ── Non-blocking PID steering state ─────────────────────
  bool  pidActive_        = false;   // true when driveStraight() is active
  int   pidBasePWM_       = 0;       // requested base speed
  long  pidStartLeft_     = 0;       // encoder snapshot at driveStraight() call
  long  pidStartRight_    = 0;
  float pidSteerIntegral_ = 0.0f;
  float pidSteerLastErr_  = 0.0f;
  unsigned long pidLastMs_     = 0;
  unsigned long pidLastPrint_  = 0;

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

  // ── Non-blocking continuous drive with PID ──────────────
  // Call driveStraight() once to start, then call updateSteering()
  // every loop iteration. PID correction keeps the robot straight.
  // Call stop() to end.
  void driveStraight(int basePWM);    // Start driving with PID
  void updateSteering();              // Call every loop — applies PID correction
  bool isPidActive() const { return pidActive_; }

  // ── Blocking distance-based moves (PID built-in) ────────
  void moveForwardCm(int cm);
  void moveBackwardCm(int cm);

  // In-place pivot turns using encoder feedback + PID.
  void turnRightDeg(int degrees);
  void turnLeftDeg(int degrees);

  void stop();
  void setMotorSpeedsPWM(int leftPWM, int rightPWM);
  void setMotorSpeedsMMS(int leftMMs, int rightMMs);
};