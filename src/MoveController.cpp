#include "MoveController.h"

// ─────────────────────────────────────────────────────────────
//  ISR support — store pin numbers in plain statics so the ISRs
//  require no pointer dereference and no function call overhead.
//
//  Direction rule for CHANGE mode (fires on both A edges):
//    if (A_state == B_state) → forward (+1)
//    else                    → backward (−1)
//  This is the standard quadrature decoder formula.
// ─────────────────────────────────────────────────────────────
static MoveController *isrInstance = nullptr;
static uint8_t _rightEncA_pin = 0;
static uint8_t _rightEncB_pin = 0;
static uint8_t _leftEncA_pin  = 0;
static uint8_t _leftEncB_pin  = 0;

static void isrRightA()
{
    bool a = digitalRead(_rightEncA_pin);
    bool b = digitalRead(_rightEncB_pin);
    if (a == b)
        isrInstance->rightPulse++;
    else
        isrInstance->rightPulse--;
}

static void isrLeftA()
{
    bool a = digitalRead(_leftEncA_pin);
    bool b = digitalRead(_leftEncB_pin);
    if (a == b)
        isrInstance->leftPulse++;
    else
        isrInstance->leftPulse--;
}

// ─────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────
MoveController::MoveController(int ena, int in1, int in2, int enb, int in3, int in4,
                               float wheelDiameterCm, int encoderPPR, float wheelBaseCm,
                               int maxSpeed_, int minSpeed_)
  : ENA(ena), IN1(in1), IN2(in2), ENB(enb), IN3(in3), IN4(in4),
    rightEncA(-1), rightEncB(-1), leftEncA(-1), leftEncB(-1),
    encoderConfigured(false),
    wheelDiameter(wheelDiameterCm), encPPR(encoderPPR), wheelBase(wheelBaseCm),
    maxSpeed(maxSpeed_), minSpeed(minSpeed_), Kp(0.45f), Ki(0.0f), Kd(0.09f) {}

void MoveController::begin()
{
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

// ─────────────────────────────────────────────────────────────
//  Encoder configuration — attach hardware interrupts
// ─────────────────────────────────────────────────────────────
void MoveController::configureEncoders(int rightA, int rightB, int leftA, int leftB)
{
    rightEncA = rightA;
    rightEncB = rightB;
    leftEncA  = leftA;
    leftEncB  = leftB;

    pinMode(rightEncA, INPUT_PULLUP);
    pinMode(rightEncB, INPUT_PULLUP);
    pinMode(leftEncA,  INPUT_PULLUP);
    pinMode(leftEncB,  INPUT_PULLUP);

    // Populate static globals for ISR direct pin access
    _rightEncA_pin = (uint8_t)rightA;
    _rightEncB_pin = (uint8_t)rightB;
    _leftEncA_pin  = (uint8_t)leftA;
    _leftEncB_pin  = (uint8_t)leftB;

    // Store instance pointer for ISRs
    isrInstance = this;

    // Attach CHANGE interrupts on channel A — fires on both rising and falling
    // edges, giving 2× counting resolution over RISING-only.
    attachInterrupt(digitalPinToInterrupt(rightEncA), isrRightA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(leftEncA),  isrLeftA,  CHANGE);

    encoderConfigured = true;
    Serial.println(F("Encoders: ISR attached (CHANGE mode)"));
}


void MoveController::resetEncoders()
{
    noInterrupts();
    rightPulse = 0;
    leftPulse  = 0;
    interrupts();
}

// ─────────────────────────────────────────────────────────────
//  Distance helpers
// ─────────────────────────────────────────────────────────────
float MoveController::pulsesToDistanceCm(long pulses) const
{
    const float PI_F = 3.14159265f;
    float wheelCirc = PI_F * wheelDiameter;
    return ((float)pulses / (float)encPPR) * wheelCirc;
}

float MoveController::getLeftDistanceCm() const
{
    return pulsesToDistanceCm(leftPulse);
}

float MoveController::getRightDistanceCm() const
{
    return pulsesToDistanceCm(rightPulse);
}

void MoveController::setPID(float Kp_, float Ki_, float Kd_)
{
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
}

// ─────────────────────────────────────────────────────────────
//  Motor control
// ─────────────────────────────────────────────────────────────
void MoveController::stop()
{
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void MoveController::setMotorSpeedsPWM(int leftPWM, int rightPWM)
{
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

void MoveController::setMotorSpeedsMMS(int leftMMs, int rightMMs)
{
    const float MMS_TO_PWM_RATIO = 255.0f / 500.0f;
    int leftPWM  = (int)(leftMMs  * MMS_TO_PWM_RATIO);
    int rightPWM = (int)(rightMMs * MMS_TO_PWM_RATIO);
    setMotorSpeedsPWM(leftPWM, rightPWM);
}

// ─────────────────────────────────────────────────────────────
//  Blocking distance-based moves (PID controlled)
//  Encoders are ISR-driven so no updateEncoderCounts() needed.
// ─────────────────────────────────────────────────────────────
void MoveController::moveForwardCm(int cm)
{
    // Snapshot current counters
    noInterrupts();
    long startRight = rightPulse;
    long startLeft  = leftPulse;
    interrupts();

    const float PI_F = 3.14159265f;
    float wheelCirc = PI_F * wheelDiameter;
    long targetPulses = (long)(((float)cm / wheelCirc) * (float)encPPR);

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    while (true) {
        // ISR updates pulses automatically — just read them
        noInterrupts();
        long rNow = rightPulse;
        long lNow = leftPulse;
        interrupts();

        long rDelta = rNow - startRight;
        long lDelta = lNow - startLeft;
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

void MoveController::moveBackwardCm(int cm)
{
    noInterrupts();
    long startRight = rightPulse;
    long startLeft  = leftPulse;
    interrupts();

    const float PI_F = 3.14159265f;
    float wheelCirc = PI_F * wheelDiameter;
    long targetPulses = (long)(((float)cm / wheelCirc) * (float)encPPR);

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    while (true) {
        noInterrupts();
        long rNow = rightPulse;
        long lNow = leftPulse;
        interrupts();

        long rDelta = abs(rNow - startRight);
        long lDelta = abs(lNow - startLeft);
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

// ─────────────────────────────────────────────────────────────
//  In-place pivot turns (PID-controlled via encoders)
//
//  Each wheel travels arc = (degrees / 360) × π × wheelBase
//  in opposite directions. The PID controls speed based on
//  how many pulses remain until the target arc is reached.
// ─────────────────────────────────────────────────────────────
void MoveController::turnRightDeg(int degrees)
{
    noInterrupts();
    long startRight = rightPulse;
    long startLeft  = leftPulse;
    interrupts();

    const float PI_F = 3.14159265f;
    // Arc each wheel must travel for this turn angle
    float arcCm = ((float)degrees / 360.0f) * PI_F * wheelBase;
    float wheelCirc = PI_F * wheelDiameter;
    long targetPulses = (long)((arcCm / wheelCirc) * (float)encPPR);

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    while (true) {
        noInterrupts();
        long lNow = leftPulse;
        long rNow = rightPulse;
        interrupts();

        // Right turn: left goes forward (+), right goes backward (−)
        long lDelta = lNow - startLeft;    // positive = forward
        long rDelta = startRight - rNow;   // positive = backward
        long avgPulses = (abs(lDelta) + abs(rDelta)) / 2;

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

        int turnSpeed = (int)output;
        if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
        if (turnSpeed < 0) turnSpeed = 0;
        if (turnSpeed < minSpeed && error > 20.0f) turnSpeed = minSpeed;

        // Left motor forward
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, turnSpeed);

        // Right motor backward
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, turnSpeed);

        delay(5);
    }

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void MoveController::turnLeftDeg(int degrees)
{
    noInterrupts();
    long startRight = rightPulse;
    long startLeft  = leftPulse;
    interrupts();

    const float PI_F = 3.14159265f;
    float arcCm = ((float)degrees / 360.0f) * PI_F * wheelBase;
    float wheelCirc = PI_F * wheelDiameter;
    long targetPulses = (long)((arcCm / wheelCirc) * (float)encPPR);

    float integral = 0.0f;
    float lastError = 0.0f;
    unsigned long lastTime = millis();

    while (true) {
        noInterrupts();
        long lNow = leftPulse;
        long rNow = rightPulse;
        interrupts();

        // Left turn: right goes forward (+), left goes backward (−)
        long rDelta = rNow - startRight;   // positive = forward
        long lDelta = startLeft - lNow;    // positive = backward
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

        int turnSpeed = (int)output;
        if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
        if (turnSpeed < 0) turnSpeed = 0;
        if (turnSpeed < minSpeed && error > 20.0f) turnSpeed = minSpeed;

        // Left motor backward
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, turnSpeed);

        // Right motor forward
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, turnSpeed);

        delay(5);
    }

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
