#include "armSlider.h"

// ─────────────────────────────────────────────────────────────
//  Calibration — adjust after physical measurement
//  Default: 1600 steps per 10 cm (standard 1/8 microstepping,
//           2 mm pitch leadscrew)
// ─────────────────────────────────────────────────────────────
static constexpr long  SLIDER_CALIB_STEPS = 2560; // steps
static constexpr float SLIDER_CALIB_CM    = 10.0f; // cm
static constexpr unsigned int PULSE_DELAY_US = 800; // µs per half-step

// Slider home is at 0 cm (left-most position).
// Positive direction moves RIGHT (away from home).
static constexpr bool DIR_RIGHT = false; // adjust if motor runs backwards
static constexpr bool DIR_LEFT  = !DIR_RIGHT;

// ─────────────────────────────────────────────────────────────
//  Internal state
// ─────────────────────────────────────────────────────────────
static float currentCm_ = 0.0f; // tracked absolute position

// ─────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────
static void enableDriver()
{
    digitalWrite(NEMMA_MOTOR_ENABLE, LOW); // LOW = enabled on most A4988/DRV8825
}

static void disableDriver()
{
    digitalWrite(NEMMA_MOTOR_ENABLE, HIGH); // HIGH = disabled
}

static long cmToSteps(float cm)
{
    if (cm < 0.0f) cm = -cm;
    return (long)((cm * (float)SLIDER_CALIB_STEPS) / SLIDER_CALIB_CM + 0.5f);
}

// Move a relative distance (positive = right, negative = left)
static void moveRelative(float deltaCm)
{
    if (deltaCm == 0.0f) return;

    bool dir = (deltaCm > 0.0f) ? DIR_RIGHT : DIR_LEFT;
    digitalWrite(NEMMA_MOTOR_DIR, dir ? HIGH : LOW);

    long steps = cmToSteps(deltaCm);

    enableDriver();

    for (long i = 0; i < steps; i++)
    {
        digitalWrite(NEMMA_MOTOR_STEP, HIGH);
        delayMicroseconds(PULSE_DELAY_US);
        digitalWrite(NEMMA_MOTOR_STEP, LOW);
        delayMicroseconds(PULSE_DELAY_US);
    }

    disableDriver();

    // Update tracked position
    if (deltaCm > 0.0f)
        currentCm_ += (float)steps * SLIDER_CALIB_CM / (float)SLIDER_CALIB_STEPS;
    else
        currentCm_ -= (float)steps * SLIDER_CALIB_CM / (float)SLIDER_CALIB_STEPS;
}

// ─────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────
void initArmSlider()
{
    // Pins are configured in main.cpp; motor is already disabled there.
    // Reset tracked position to home.
    currentCm_ = 0.0f;
    Serial.println(F("ArmSlider ready"));
}

void armSliderMoveTo(float targetCm)
{
    float delta = targetCm - currentCm_;
    if (delta == 0.0f) return;

    Serial.print(F("  [Slider] "));
    Serial.print(currentCm_, 1);
    Serial.print(F(" -> "));
    Serial.print(targetCm, 1);
    Serial.println(F(" cm"));

    moveRelative(delta);
}

void armSliderReturn()
{
    Serial.println(F("  [Slider] Returning to home (0 cm)"));
    armSliderMoveTo(0.0f);
}

float armSliderPositionCm()
{
    return currentCm_;
}
