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

static constexpr bool PID_DEBUG_PRINTS = false;
static constexpr unsigned long PID_PRINT_INTERVAL_MS = 100UL;

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
    maxSpeed(maxSpeed_), minSpeed(minSpeed_), Kp(0.3f), Ki(0.0f), Kd(0.035f) {}

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
    pidActive_ = false;

    // Active Braking:
    // Set IN pins to SAME value (LOW/LOW) to short motor terminals to Ground.
    // Set Enable pins HIGH to activate the H-bridge switches.
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);

    // Wait briefly for kinetic energy to dissipate
    delay(150);

    // Now disable motors to save power
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

// ─────────────────────────────────────────────────────────────
//  Non-blocking continuous drive with PID straight-line correction.
//
//  Call driveStraight(basePWM) once to start, then call
//  updateSteering() every loop iteration.
//  Call stop() to end.
// ─────────────────────────────────────────────────────────────
void MoveController::driveStraight(int basePWM)
{
    // Snapshot encoder positions
    noInterrupts();
    pidStartLeft_  = leftPulse;
    pidStartRight_ = rightPulse;
    interrupts();

    pidBasePWM_       = basePWM;
    pidSteerIntegral_ = 0.0f;
    pidSteerLastErr_  = 0.0f;
    pidLastMs_        = millis();
    pidLastPrint_     = 0;
    pidActive_        = true;

    // Set initial motor direction (forward if positive, backward if negative)
    if (basePWM >= 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    analogWrite(ENA, abs(basePWM));
    analogWrite(ENB, abs(basePWM));
}

void MoveController::updateSteering()
{
    if (!pidActive_ || !encoderConfigured) return;

    unsigned long now = millis();
    float dt = ((float)(now - pidLastMs_)) / 1000.0f;
    pidLastMs_ = now;
    if (dt <= 0.0f) return;        // too fast, skip
    if (dt > 0.2f) dt = 0.2f;     // cap large gaps

    // Read encoder deltas
    noInterrupts();
    long lNow = leftPulse;
    long rNow = rightPulse;
    interrupts();

    long lDelta = abs(lNow - pidStartLeft_);
    long rDelta = abs(rNow - pidStartRight_);

    // Steering error: positive → left has turned more → slow left, speed up right
    float steerError = (float)(lDelta - rDelta);

    pidSteerIntegral_ += steerError * dt;
    // Anti-windup
    float maxI = (float)maxSpeed * 0.5f;
    if (pidSteerIntegral_ >  maxI) pidSteerIntegral_ =  maxI;
    if (pidSteerIntegral_ < -maxI) pidSteerIntegral_ = -maxI;

    float steerDerivative = (steerError - pidSteerLastErr_) / dt;
    pidSteerLastErr_ = steerError;

    float correction = Kp * steerError + Ki * pidSteerIntegral_ + Kd * steerDerivative;

    int base = abs(pidBasePWM_);
    int leftSpeed  = base - (int)correction;
    int rightSpeed = base + (int)correction;
    leftSpeed  = constrain(leftSpeed,  0, maxSpeed);
    rightSpeed = constrain(rightSpeed, 0, maxSpeed);

    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);

    // Debug prints
    if (PID_DEBUG_PRINTS && (now - pidLastPrint_) >= PID_PRINT_INTERVAL_MS) {
        pidLastPrint_ = now;
        Serial.print(F("[PID DRV] err="));  Serial.print(steerError, 1);
        Serial.print(F(" corr="));          Serial.print(correction, 1);
        Serial.print(F(" L="));             Serial.print(leftSpeed);
        Serial.print(F(" R="));             Serial.print(rightSpeed);
        Serial.print(F(" lP="));            Serial.print(lDelta);
        Serial.print(F(" rP="));            Serial.println(rDelta);
    }
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

    // PID state for straight-line correction
    float steerIntegral  = 0.0f;
    float steerLastError = 0.0f;
    unsigned long lastTime = millis();
    unsigned long lastPidPrint = 0;

    // Slow-down zone: start decelerating this many pulses before target
    long slowZonePulses = targetPulses / 4;
    if (slowZonePulses < 10) slowZonePulses = 10;

    while (true) {
        noInterrupts();
        long rNow = rightPulse;
        long lNow = leftPulse;
        interrupts();

        long rDelta = rNow - startRight;
        long lDelta = lNow - startLeft;
        long avgPulses = (abs(rDelta) + abs(lDelta)) / 2;

        // ── Stop condition ─────────────────────────────────
        if (avgPulses >= targetPulses) break;

        // ── Base speed (distance-based deceleration) ───────
        long remaining = targetPulses - avgPulses;
        int baseSpeed;
        if (remaining <= slowZonePulses) {
            // Linear ramp from maxSpeed down to minSpeed
            baseSpeed = (int)map(remaining, 0, slowZonePulses, minSpeed, maxSpeed);
            if (baseSpeed < minSpeed) baseSpeed = minSpeed;
        } else {
            baseSpeed = maxSpeed;
        }

        // ── Steering PID (straight-line correction) ────────
        // Error = how much left has turned more than right.
        // Positive error → left ahead → slow left, speed up right.
        float steerError = (float)(lDelta - rDelta);

        unsigned long now = millis();
        float dt = ((float)(now - lastTime)) / 1000.0f;
        lastTime = now;
        if (dt <= 0.0f) dt = 0.001f;
        if (dt > 0.2f) dt = 0.2f;

        steerIntegral += steerError * dt;
        // Anti-windup
        float maxI = (float)maxSpeed * 0.5f;
        if (steerIntegral >  maxI) steerIntegral =  maxI;
        if (steerIntegral < -maxI) steerIntegral = -maxI;

        float steerDerivative = (steerError - steerLastError) / dt;
        steerLastError = steerError;

        float correction = Kp * steerError + Ki * steerIntegral + Kd * steerDerivative;

        // ── Apply: add correction to lagging, subtract from leading ─
        int leftSpeed  = baseSpeed - (int)correction;
        int rightSpeed = baseSpeed + (int)correction;

        // Clamp
        leftSpeed  = constrain(leftSpeed,  0, maxSpeed);
        rightSpeed = constrain(rightSpeed, 0, maxSpeed);

        if (PID_DEBUG_PRINTS && (now - lastPidPrint) >= PID_PRINT_INTERVAL_MS) {
            lastPidPrint = now;
            Serial.print(F("[PID FWD] err="));   Serial.print(steerError, 2);
            Serial.print(F(" corr="));           Serial.print(correction, 2);
            Serial.print(F(" base="));           Serial.print(baseSpeed);
            Serial.print(F(" L="));              Serial.print(leftSpeed);
            Serial.print(F(" R="));              Serial.print(rightSpeed);
            Serial.print(F(" avgP="));           Serial.print(avgPulses);
            Serial.print(F("/"));                Serial.println(targetPulses);
        }

        // Left motor forward
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, leftSpeed);

        // Right motor forward
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, rightSpeed);

        delay(5);
    }
    stop();

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

    float steerIntegral  = 0.0f;
    float steerLastError = 0.0f;
    unsigned long lastTime = millis();
    unsigned long lastPidPrint = 0;

    long slowZonePulses = targetPulses / 4;
    if (slowZonePulses < 10) slowZonePulses = 10;

    while (true) {
        noInterrupts();
        long rNow = rightPulse;
        long lNow = leftPulse;
        interrupts();

        long rDelta = abs(rNow - startRight);
        long lDelta = abs(lNow - startLeft);
        long avgPulses = (rDelta + lDelta) / 2;

        if (avgPulses >= targetPulses) break;

        // Base speed with deceleration
        long remaining = targetPulses - avgPulses;
        int baseSpeed;
        if (remaining <= slowZonePulses) {
            baseSpeed = (int)map(remaining, 0, slowZonePulses, minSpeed, maxSpeed);
            if (baseSpeed < minSpeed) baseSpeed = minSpeed;
        } else {
            baseSpeed = maxSpeed;
        }

        // Steering PID — keep straight while reversing
        float steerError = (float)(lDelta - rDelta);

        unsigned long now = millis();
        float dt = ((float)(now - lastTime)) / 1000.0f;
        lastTime = now;
        if (dt <= 0.0f) dt = 0.001f;
        if (dt > 0.2f) dt = 0.2f;

        steerIntegral += steerError * dt;
        float maxI = (float)maxSpeed * 0.5f;
        if (steerIntegral >  maxI) steerIntegral =  maxI;
        if (steerIntegral < -maxI) steerIntegral = -maxI;

        float steerDerivative = (steerError - steerLastError) / dt;
        steerLastError = steerError;

        float correction = Kp * steerError + Ki * steerIntegral + Kd * steerDerivative;

        int leftSpeed  = baseSpeed - (int)correction;
        int rightSpeed = baseSpeed + (int)correction;
        leftSpeed  = constrain(leftSpeed,  0, maxSpeed);
        rightSpeed = constrain(rightSpeed, 0, maxSpeed);

        if (PID_DEBUG_PRINTS && (now - lastPidPrint) >= PID_PRINT_INTERVAL_MS) {
            lastPidPrint = now;
            Serial.print(F("[PID REV] err="));   Serial.print(steerError, 2);
            Serial.print(F(" corr="));           Serial.print(correction, 2);
            Serial.print(F(" base="));           Serial.print(baseSpeed);
            Serial.print(F(" L="));              Serial.print(leftSpeed);
            Serial.print(F(" R="));              Serial.print(rightSpeed);
            Serial.print(F(" avgP="));           Serial.print(avgPulses);
            Serial.print(F("/"));                Serial.println(targetPulses);
        }

        // Left motor backward
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);

        // Right motor backward
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);

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
    unsigned long lastPidPrint = 0;

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

        if (PID_DEBUG_PRINTS && (now - lastPidPrint) >= PID_PRINT_INTERVAL_MS) {
            lastPidPrint = now;
            Serial.print(F("[PID TR] err="));    Serial.print(error, 2);
            Serial.print(F(" out="));            Serial.print(output, 2);
            Serial.print(F(" spd="));            Serial.print(turnSpeed);
            Serial.print(F(" avgP="));           Serial.print(avgPulses);
            Serial.print(F("/"));                Serial.println(targetPulses);
        }

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
    stop();

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
    unsigned long lastPidPrint = 0;

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

        if (PID_DEBUG_PRINTS && (now - lastPidPrint) >= PID_PRINT_INTERVAL_MS) {
            lastPidPrint = now;
            Serial.print(F("[PID TL] err="));    Serial.print(error, 2);
            Serial.print(F(" out="));            Serial.print(output, 2);
            Serial.print(F(" spd="));            Serial.print(turnSpeed);
            Serial.print(F(" avgP="));           Serial.print(avgPulses);
            Serial.print(F("/"));                Serial.println(targetPulses);
        }

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
    stop();

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
