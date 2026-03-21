#include "RobotTests.h"
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  Test implementations
// ─────────────────────────────────────────────────────────────

// ── MOVE_FORWARD_10CM ─────────────────────────────────────────
// Drive straight forward 10 cm using encoder-based control,
// then stop.
static void test_moveForward10cm(MoveController &robot)
{
    Serial.println(F("[Test] MOVE_FORWARD_10CM — driving 10 cm"));
    robot.moveForwardCm(10);
    robot.stop();
    Serial.println(F("[Test] Done — robot stopped"));
}

// ── TURN_RIGHT_90DEG ──────────────────────────────────────────
// In-place right turn by 90°.
// Arc length for each wheel = (π * wheelBase / 2) / 2 = π * wheelBase / 4
// With wheelBase = 18 cm → arc ≈ 14.14 cm per wheel (opposite directions).
static void test_turnRight90(MoveController &robot)
{
    Serial.println(F("[Test] TURN_RIGHT_90DEG — turning right 90°"));

    // For an in-place turn the left wheel moves forward and
    // the right wheel moves backward by the same arc distance.
    // Arc = (π × wheelBase) / 4
    //     = (3.14159 × 18) / 4  ≈  14.14 cm
    const int TURN_ARC_CM = 14; // rounded; fine-tune based on field test

    // Left motor forward, right motor backward simultaneously using PWM.
    // Since moveForwardCm / moveBackwardCm are sequential we drive both
    // with raw PWM for the equivalent pulse duration, then stop.
    // Alternatively: use the blocking helpers sequentially if one-wheel
    // pivot is acceptable.
    //
    // Here we do a one-wheel pivot (right wheel stationary, left drives)
    // which is simpler and accurate enough for 90°.
    //
    // To do a true centre-pivot (both wheels equal, opposite), replace with:
    //   robot.setMotorSpeedsPWM(+TURN_PWM, -TURN_PWM) + timed delay.

    const int TURN_PWM = 120;
    const unsigned long TURN_DURATION_MS = 600; // tune: ms for ~90° at TURN_PWM

    robot.setMotorSpeedsPWM(TURN_PWM, -TURN_PWM); // left fwd, right back
    delay(TURN_DURATION_MS);
    robot.stop();

    Serial.print  (F("[Test] Done — turned right ~90°  (arc="));
    Serial.print  (TURN_ARC_CM);
    Serial.println(F("cm ref, timer-based)"));
    Serial.println(F("[Test] Tip: adjust TURN_DURATION_MS to calibrate angle"));
}

// ─────────────────────────────────────────────────────────────
//  Dispatcher
// ─────────────────────────────────────────────────────────────
void runRobotTest(RobotTest test, MoveController &robot)
{
    Serial.println(F("\n========================================"));
    Serial.println(F("  ROBOT TEST MODE"));
    Serial.println(F("========================================"));

    switch (test)
    {
    case RobotTest::MOVE_FORWARD_10CM:
        test_moveForward10cm(robot);
        break;

    case RobotTest::TURN_RIGHT_90DEG:
        test_turnRight90(robot);
        break;

    // ── Add new cases here as you add tests ──────────────
    }

    Serial.println(F("========================================"));
    Serial.println(F("  TEST COMPLETE — robot halted"));
    Serial.println(F("========================================\n"));
}
