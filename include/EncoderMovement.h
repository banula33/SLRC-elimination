#ifndef ENCODER_MOVEMENT_H
#define ENCODER_MOVEMENT_H

#include <Arduino.h>
#include <math.h>

// ─────────────────────────────────────────────
//  Robot Distance Control — Encoder Based
//  Left Motor:  ENA=2, IN1=4, IN2=5  | Encoder A=27, B=29
//  Right Motor: ENB=3, IN3=6, IN4=7  | Encoder A=23, B=25
//  Wheel Diameter: 6.5cm | PPR: 1550
// ─────────────────────────────────────────────

// ── PARAMETER: Change this to move a different distance ──
extern const float TARGET_DISTANCE_CM;   // <-- set your desired distance here
// ─────────────────────────────────────────────────────────

// Motor pins
#define ENA 2
#define ENB 3
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

// Encoder pins
#define LEFT_ENC_A  27
#define LEFT_ENC_B  29
#define RIGHT_ENC_A 23
#define RIGHT_ENC_B 25

// Wheel & encoder specs
extern const float WHEEL_DIAMETER_CM;
extern const float PPR;
extern const float WHEEL_CIRCUM_CM;

// Motor base speed (0–255). Lower = more accurate stopping
extern const int BASE_SPEED;
extern const int SLOW_SPEED;
extern const int SLOW_ZONE_CM;

// Encoder counts
extern volatile long leftPulses;
extern volatile long rightPulses;
extern int lastLeftA;
extern int lastRightA;

// ── Helper functions ───────────────────────
float pulsesToCm(long pulses);
long cmToPulses(float cm);

// ── Motor control ──────────────────────────
void motorLeft(int speed);
void motorRight(int speed);
void stopMotors();

// ── Encoder reading ────────────────────────
void readEncoders();

// ── Main movement function ─────────────────
void moveDistance(float distance_cm);

#endif
