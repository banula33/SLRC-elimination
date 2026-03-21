#pragma once

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  GripperArm — servo-driven vertical lift arm + gripper claw.
//
//  Arm servo (pin 39):  0° = fully up (home), 180° = fully down
//  Gripper servo (pin 41): 0° = closed (grip), 180° = open
//
//  Tunable angles — adjust these to match your physical setup.
// ─────────────────────────────────────────────────────────────

// Arm angles
static constexpr int ARM_UP_DEG      = 0;    // Hook retracted / home position
static constexpr int ARM_DOWN_DEG    = 180;  // Hook fully lowered onto box
static constexpr int ARM_PARTIAL_DEG = 90;   // Partially lowered over storage slot

// Gripper angles
static constexpr int GRIPPER_OPEN_DEG  = 180; // Hook expanded
static constexpr int GRIPPER_CLOSE_DEG = 0;   // Hook gripping box

// Call once from setup()
void initGripperArm(uint8_t armPin = 45, uint8_t gripperPin = 44);

// Smoothly sweep arm servo from fromDeg to toDeg
void moveArmSmooth(int fromDeg, int toDeg, int dlyMs = 15);

// Gripper helpers
void gripperOpen();   // GRIPPER_OPEN_DEG
void gripperClose();  // GRIPPER_CLOSE_DEG

// Returns the last position the arm was commanded to
int currentArmDeg();