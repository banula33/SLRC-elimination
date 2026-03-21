#pragma once

#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  ArmSlider — NEMA stepper-driven horizontal slider.
//
//  The motor driver is kept DISABLED (EN pin HIGH) when idle
//  to save current. It is enabled only for the duration of a
//  move and then disabled immediately after.
//
//  Pins are taken from main.cpp globals (extern).
// ─────────────────────────────────────────────────────────────

extern const uint8_t NEMMA_MOTOR_ENABLE;
extern const uint8_t NEMMA_MOTOR_STEP;
extern const uint8_t NEMMA_MOTOR_DIR;

// Call once from setup() — pins already configured in main.cpp.
void initArmSlider();

// Move slider to an absolute position (cm from home/left end).
// Tracks current position internally and moves the delta.
// Enables the driver, steps, then disables the driver.
void armSliderMoveTo(float targetCm);

// Return slider to the home position (0 cm).
void armSliderReturn();

// Query current slider position (cm).
float armSliderPositionCm();
