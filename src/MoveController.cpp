#include "MoveController.h"

MoveController::MoveController(int ena, int in1, int in2, int enb, int in3, int in4,
                               float wheelDiameterCm, int encoderPPR, float wheelBaseCm,
                               int maxSpeed_, int minSpeed_)
  : ENA(ena), IN1(in1), IN2(in2), ENB(enb), IN3(in3), IN4(in4),
    rightEncA(-1), rightEncB(-1), leftEncA(-1), leftEncB(-1),
    lastRightA(LOW), lastLeftA(LOW), encoderConfigured(false),
    wheelDiameter(wheelDiameterCm), encPPR(encoderPPR), wheelBase(wheelBaseCm),
    maxSpeed(maxSpeed_), minSpeed(minSpeed_), Kp(0.45f), Ki(0.0f), Kd(0.09f) {}

void MoveController::begin() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void MoveController::setPID(float Kp_, float Ki_, float Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void MoveController::configureEncoders(int rightA, int rightB, int leftA, int leftB) {
  rightEncA = rightA;
  rightEncB = rightB;
  leftEncA = leftA;
  leftEncB = leftB;

  pinMode(rightEncA, INPUT_PULLUP);
  pinMode(rightEncB, INPUT_PULLUP);
  pinMode(leftEncA, INPUT_PULLUP);
  pinMode(leftEncB, INPUT_PULLUP);

  lastRightA = digitalRead(rightEncA);
  lastLeftA = digitalRead(leftEncA);
  encoderConfigured = true;
}

void MoveController::pollEncoders() {
  updateEncoderCounts();
}

float MoveController::pulsesToDistanceCm(long pulses) const {
  const float PI_F = 3.14159265f;
  float wheelCirc = PI_F * wheelDiameter;
  return ((float)pulses / (float)encPPR) * wheelCirc;
}

float MoveController::getLeftDistanceCm() const {
  return pulsesToDistanceCm(leftPulse);
}

float MoveController::getRightDistanceCm() const {
  return pulsesToDistanceCm(rightPulse);
}

void MoveController::updateEncoderCounts() {
  if (!encoderConfigured) return;

  int currRightA = digitalRead(rightEncA);
  int currLeftA = digitalRead(leftEncA);

  // Count one pulse per rising edge on channel A.
  if (lastRightA == LOW && currRightA == HIGH) {
    rightPulse++;
  }
  if (lastLeftA == LOW && currLeftA == HIGH) {
    leftPulse++;
  }

  lastRightA = currRightA;
  lastLeftA = currLeftA;
}

void MoveController::stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void MoveController::moveForwardCm(int cm) {
  long startRight = rightPulse;
  long startLeft = leftPulse;

  const float PI_F = 3.14159265f;
  float wheelCirc = PI_F * wheelDiameter;
  long targetPulses = (long)(((float)cm / wheelCirc) * (float)encPPR);

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  while (true) {
    updateEncoderCounts();

    long rDelta = rightPulse - startRight;
    long lDelta = leftPulse - startLeft;

    long avgPulses = (abs(rDelta) + abs(lDelta)) / 2;

    float error = (float)(targetPulses - avgPulses);
    if (error <= 0.0f) break;

    unsigned long now = millis();
    float dt = ((float)(now - lastTime)) / 1000.0f;
    lastTime = now;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.2f) dt = 0.2f;

    integral += error * dt;
    float maxIntegral = (float)maxSpeed * 2.0f;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int forwardSpeed = (int)output;
    if (forwardSpeed > maxSpeed) forwardSpeed = maxSpeed;
    if (forwardSpeed < 0) forwardSpeed = 0;
    if (forwardSpeed < minSpeed && error > 20.0f) forwardSpeed = minSpeed;

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, forwardSpeed);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, forwardSpeed);

    delay(5);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void MoveController::moveBackwardCm(int cm) {
  long startRight = rightPulse;
  long startLeft = leftPulse;

  const float PI_F = 3.14159265f;
  float wheelCirc = PI_F * wheelDiameter;
  long targetPulses = (long)(((float)cm / wheelCirc) * (float)encPPR);

  float integral = 0.0f;
  float lastError = 0.0f;
  unsigned long lastTime = millis();

  while (true) {
    updateEncoderCounts();

    long rDelta = abs(rightPulse - startRight);
    long lDelta = abs(leftPulse - startLeft);

    long avgPulses = (rDelta + lDelta) / 2;

    float error = (float)(targetPulses - avgPulses);
    if (error <= 0.0f) break;

    unsigned long now = millis();
    float dt = ((float)(now - lastTime)) / 1000.0f;
    lastTime = now;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.2f) dt = 0.2f;

    integral += error * dt;
    float maxIntegral = (float)maxSpeed * 2.0f;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;

    float derivative = (error - lastError) / dt;
    lastError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int backwardSpeed = (int)output;
    if (backwardSpeed > maxSpeed) backwardSpeed = maxSpeed;
    if (backwardSpeed < 0) backwardSpeed = 0;
    if (backwardSpeed < minSpeed && error > 20.0f) backwardSpeed = minSpeed;

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, backwardSpeed);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, backwardSpeed);

    delay(5);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void MoveController::setMotorSpeedsPWM(int leftPWM, int rightPWM) {
  // Left Motor (A)
  if (leftPWM > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, min(leftPWM, maxSpeed));
  } else if (leftPWM < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, min(-leftPWM, maxSpeed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // Right Motor (B)
  if (rightPWM > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, min(rightPWM, maxSpeed));
  } else if (rightPWM < 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, min(-rightPWM, maxSpeed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void MoveController::setMotorSpeedsMMS(int leftMMs, int rightMMs) {
  // Simple Open-Loop Mapping (mm/s to PWM)
  // E.g. Assume maxSpeed (255) == 500 mm/s. Adjust this ratio to match your robot!
  const float MMS_TO_PWM_RATIO = 255.0f / 500.0f; 
  
  int leftPWM = (int)(leftMMs * MMS_TO_PWM_RATIO);
  int rightPWM = (int)(rightMMs * MMS_TO_PWM_RATIO);
  
  setMotorSpeedsPWM(leftPWM, rightPWM);
}
