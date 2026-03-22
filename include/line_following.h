#pragma once
#include <Arduino.h>
#include <QTRSensors.h>

// ─────────────────────────────────────────────────────────────
//  Line sensor module — 8-channel QTR-RC array.
//
//  Provides init, calibration, and read helpers.
//  Motor control is handled by MoveController, not here.
// ─────────────────────────────────────────────────────────────

// Sensor pin assignments (digital pins on Mega)
static constexpr uint8_t LINE_SENSOR_COUNT = 8;
static const uint8_t LINE_SENSOR_PINS[LINE_SENSOR_COUNT] = {46, 47, 48, 49, 50, 51, 52, 53};

// Centre position for PID line-following (0–7000 for 8 sensors)
static constexpr int32_t LINE_CENTER_POSITION = 3500;

// Threshold: if any calibrated sensor reads above this,
// we consider that sensor is seeing the "line" colour.
static constexpr uint16_t LINE_DETECT_THRESHOLD = 800;

// ── Line colour configuration ─────────────────────────────
// Set to true  → line is WHITE on a dark background
// Set to false → line is BLACK on a light background
// Change this single flag to switch between the two modes.
static constexpr bool LINE_IS_WHITE = true;

// ── Lifecycle ─────────────────────────────────────────────────
// Call once from setup() to configure sensor pins.
void initLineSensors();

// Blocking calibration — manually move the robot over BOTH the
// line colour AND the background colour during this call.
// Should be called in setup() so the user can sweep by hand.
// Duration configurable (default 5 seconds).
void calibrateLineSensors(unsigned long durationMs = 5000);

// Returns true if calibration has been completed.
bool lineSensorsCalibrated();

// ── Reading ───────────────────────────────────────────────────
// Returns calibrated line position (0–7000).
// Respects LINE_IS_WHITE — uses readLineWhite or readLineBlack.
uint16_t readLinePosition(uint16_t *sensorValues = nullptr);

// Returns true if any sensor detects the line.
bool isOnLine();

// Returns true when ALL sensors see the line simultaneously —
// indicates a full horizontal white bar crossing the path.
// Use to detect the stop zone at end of line-follow run.
bool isFullWhiteBar();

// Read raw calibrated values into a caller-supplied array[8].
// Returns false if not calibrated.
bool readLineSensorRaw(uint16_t *out);

// Print calibration min/max summary to Serial.
void printLineSensorCalibration();
