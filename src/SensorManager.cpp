#include "SensorManager.h"
#include <Wire.h>

// MPU-6050 I2C address
static constexpr uint8_t MPU_ADDR = 0x68;

SensorManager::SensorManager(MoveController &robot, ToFArray &tofArray)
    : robot_(robot), tofArray_(tofArray) {}

// Put the MPU-6050 into sleep mode so it doesn't interfere with the
// I2C bus while we're only using ToF sensors.  An awake but un-polled
// MPU can hold SDA low or send rogue ACKs, causing random ToF errors.
static void sleepMPU()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x40);  // Set SLEEP bit (bit 6)
    Wire.endTransmission(true);
}

void SensorManager::begin()
{
    Wire.begin();
    Wire.setClock(400000);

    // Force the MPU-6050 to sleep — keeps the I2C bus clean for ToF.
    // When gyro is enabled later, setupRawGyro() will wake it up.
    sleepMPU();

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

    // --- Encoders ---
    // ISR-driven: pulses are counted by hardware interrupts.
    // encodersEnabled_ flag is used by the state machine to
    // decide whether to read encoder values, but no polling needed.

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
void SensorManager::enableIrArray(bool on)   { irArrayEnabled_  = on; }

void SensorManager::enableGyro(bool on)
{
    if (on && !gyroEnabled_)
    {
        // Wake MPU-6050 and configure it
        setupRawGyro();
    }
    else if (!on && gyroEnabled_)
    {
        // Put MPU-6050 back to sleep to keep I2C bus clean
        sleepMPU();
    }
    gyroEnabled_ = on;
}

void SensorManager::disableAll()
{
    encodersEnabled_ = false;
    tofLeftEnabled_  = false;
    tofFrontEnabled_ = false;
    tofRightEnabled_ = false;
    enableGyro(false);  // also puts MPU to sleep if it was awake
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
