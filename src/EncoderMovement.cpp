#include "EncoderMovement.h"

// ── PARAMETER: Change this to move a different distance ──
const float TARGET_DISTANCE_CM = 20.0;   // <-- set your desired distance here
// ─────────────────────────────────────────────────────────

// Wheel & encoder specs
const float WHEEL_DIAMETER_CM = 6.5;
const float PPR               = 830.0;
const float WHEEL_CIRCUM_CM   = PI * WHEEL_DIAMETER_CM;  // ~20.42 cm

// Motor base speed (0–255). Lower = more accurate stopping
const int BASE_SPEED    = 150;
const int SLOW_SPEED    = 100;   // slows down near target
const int SLOW_ZONE_CM  = 2.0;   // start slowing this many cm before target

// Encoder counts
volatile long leftPulses  = 0;
volatile long rightPulses = 0;
int lastLeftA  = LOW;
int lastRightA = LOW;

// ── Helper: pulses → cm ─────────────────────────────────
float pulsesToCm(long pulses) {
  return (abs(pulses) / PPR) * WHEEL_CIRCUM_CM;
}

// ── Helper: cm → pulses ─────────────────────────────────
long cmToPulses(float cm) {
  return (long)((cm / WHEEL_CIRCUM_CM) * PPR);
}

// ── Motor control ───────────────────────────────────────
void motorLeft(int speed) {
  // positive = forward, negative = backward, 0 = stop
  if (speed > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  }
  analogWrite(ENA, constrain(abs(speed), 0, 255));
}

void motorRight(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, constrain(abs(speed), 0, 255));
}

void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

// ── Read encoders (polling) ─────────────────────────────
void readEncoders() {
  int leftA  = digitalRead(LEFT_ENC_A);
  int leftB  = digitalRead(LEFT_ENC_B);
  int rightA = digitalRead(RIGHT_ENC_A);
  int rightB = digitalRead(RIGHT_ENC_B);

  if (leftA != lastLeftA) {
    (leftA != leftB) ? leftPulses++ : leftPulses--;
    lastLeftA = leftA;
  }
  if (rightA != lastRightA) {
    (rightA != rightB) ? rightPulses++ : rightPulses--;
    lastRightA = rightA;
  }
}

// ── MAIN FUNCTION: Move exact distance ─────────────────
// distance_cm: positive = forward, negative = backward
void moveDistance(float distance_cm) {
  // Reset counters
  leftPulses  = 0;
  rightPulses = 0;
  lastLeftA  = digitalRead(LEFT_ENC_A);
  lastRightA = digitalRead(RIGHT_ENC_A);

  long targetPulses = cmToPulses(abs(distance_cm));
  int  direction    = (distance_cm >= 0) ? 1 : -1;

  Serial.print("Moving ");
  Serial.print(distance_cm);
  Serial.print(" cm  →  target pulses: ");
  Serial.println(targetPulses);

  motorLeft(direction  * BASE_SPEED);
  motorRight(direction * BASE_SPEED);

  while (true) {
    readEncoders();

    long leftAbs  = abs(leftPulses);
    long rightAbs = abs(rightPulses);
    long avgPulses = (leftAbs + rightAbs) / 2;

    float distanceTravelled = pulsesToCm(avgPulses);
    float remaining         = abs(distance_cm) - distanceTravelled;

    // ── Slow down zone ──────────────────────────────────
    int speed;
    if (remaining <= SLOW_ZONE_CM) {
      // Linearly scale speed from SLOW_SPEED down to 60
      speed = map((long)remaining, 0, (long)SLOW_ZONE_CM, 60, SLOW_SPEED);
      speed = constrain(speed, 60, SLOW_SPEED);
    } else {
      speed = BASE_SPEED;
    }

    // ── Straight correction ─────────────────────────────
    // If one wheel is ahead, slow it down slightly
    long diff = leftAbs - rightAbs;
    int leftSpeed  = speed - (diff >  5 ?  15 : 0);
    int rightSpeed = speed - (diff < -5 ? 15 : 0);

    motorLeft (direction * leftSpeed);
    motorRight(direction * rightSpeed);

    // ── Stop condition ──────────────────────────────────
    if (avgPulses >= targetPulses) {
      stopMotors();
      Serial.print("Reached! Distance travelled: ");
      Serial.print(distanceTravelled, 2);
      Serial.print(" cm  |  L pulses: ");
      Serial.print(leftAbs);
      Serial.print("  R pulses: ");
      Serial.println(rightAbs);
      break;
    }
  }
}
