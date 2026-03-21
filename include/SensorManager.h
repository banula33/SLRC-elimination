#pragma once

#include <Arduino.h>
#include "ToFArray.h"
#include "RawGyro.h"
#include "MoveController.h"

// ─────────────────────────────────────────────────────────────
//  SensorManager — selective sensor polling to save CPU cycles.
//  Each phase enables only the sensor groups it needs.
// ─────────────────────────────────────────────────────────────

class SensorManager
{
public:
    SensorManager(MoveController &robot, ToFArray &tofArray);

    // ── Lifecycle ──────────────────────────────────────────
    void begin();   // Call once in setup()
    void update();  // Call every loop() — polls only enabled groups

    // ── Enable / Disable sensor groups ─────────────────────
    void enableEncoders(bool on);
    void enableTofLeft(bool on);
    void enableTofFront(bool on);
    void enableTofRight(bool on);
    void enableGyro(bool on);
    void enableIrArray(bool on);

    // Convenience: disable everything at once
    void disableAll();

    // ── Latest cached readings ─────────────────────────────
    // ToF (millimetres, -1 = invalid)
    int getLeftTof()  const { return lastLeftTof_; }
    int getFrontTof() const { return lastFrontTof_; }
    int getRightTof() const { return lastRightTof_; }

    // Encoder distances (cm)
    float getLeftEncoderCm()  const;
    float getRightEncoderCm() const;

    // Raw gyro
    const RawGyroReadings &getGyro() const { return lastGyro_; }

    // IR array (placeholder — values not yet polled)
    const uint8_t *getIrArray() const { return irValues_; }

private:
    // Hardware references
    MoveController &robot_;
    ToFArray       &tofArray_;

    // Enable flags
    bool encodersEnabled_ = false;
    bool tofLeftEnabled_   = false;
    bool tofFrontEnabled_  = false;
    bool tofRightEnabled_  = false;
    bool gyroEnabled_      = false;
    bool irArrayEnabled_   = false;

    // Timing
    static constexpr unsigned long GYRO_INTERVAL_MS = 5;
    static constexpr unsigned long TOF_INTERVAL_MS  = 120;
    unsigned long lastGyroMs_ = 0;
    unsigned long lastTofMs_  = 0;

    // Cached readings
    int lastLeftTof_  = -1;
    int lastFrontTof_ = -1;
    int lastRightTof_ = -1;
    RawGyroReadings lastGyro_ = {0, 0, 0, 0, 0, 0, 0};
    uint8_t irValues_[8] = {0};
};
