#include "SensorManager.h"

SensorManager::SensorManager(MoveController &robot, ToFArray &tofArray)
    : robot_(robot), tofArray_(tofArray) {}

void SensorManager::begin()
{
    setupRawGyro();

    if (tofArray_.begin())
    {
        Serial.println(F("ToF sensors ready"));
    }
    else
    {
        Serial.println(F("ToF init failed"));
    }
}

// ── Called every loop() ────────────────────────────────────
void SensorManager::update()
{
    unsigned long now = millis();

    // --- Encoders (polled every cycle when enabled) ---
    if (encodersEnabled_)
    {
        robot_.pollEncoders();
    }

    // --- Gyro (on interval) ---
    if (gyroEnabled_ && (now - lastGyroMs_) >= GYRO_INTERVAL_MS)
    {
        RawGyroReadings g;
        if (updateRawGyro(g))
        {
            lastGyro_ = g;
        }
        lastGyroMs_ = now;
    }

    // --- ToF sensors (on interval, only enabled ones) ---
    if ((tofLeftEnabled_ || tofFrontEnabled_ || tofRightEnabled_) &&
        (now - lastTofMs_) >= TOF_INTERVAL_MS)
    {
        if (tofLeftEnabled_)
            lastLeftTof_ = tofArray_.readLeftMm();
        if (tofFrontEnabled_)
            lastFrontTof_ = tofArray_.readFrontMm();
        if (tofRightEnabled_)
            lastRightTof_ = tofArray_.readRightMm();

        lastTofMs_ = now;
    }

    // --- IR Array (placeholder — add actual polling here later) ---
    // if (irArrayEnabled_) { ... }
}

// ── Enable / disable helpers ──────────────────────────────
void SensorManager::enableEncoders(bool on) { encodersEnabled_ = on; }
void SensorManager::enableTofLeft(bool on)   { tofLeftEnabled_  = on; }
void SensorManager::enableTofFront(bool on)  { tofFrontEnabled_ = on; }
void SensorManager::enableTofRight(bool on)  { tofRightEnabled_ = on; }
void SensorManager::enableGyro(bool on)      { gyroEnabled_     = on; }
void SensorManager::enableIrArray(bool on)   { irArrayEnabled_  = on; }

void SensorManager::disableAll()
{
    encodersEnabled_ = false;
    tofLeftEnabled_  = false;
    tofFrontEnabled_ = false;
    tofRightEnabled_ = false;
    gyroEnabled_     = false;
    irArrayEnabled_  = false;
}

// ── Cached encoder distances ──────────────────────────────
float SensorManager::getLeftEncoderCm() const
{
    return robot_.getLeftDistanceCm();
}

float SensorManager::getRightEncoderCm() const
{
    return robot_.getRightDistanceCm();
}
