#include "RobotTests.h"
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  Test implementations
// ─────────────────────────────────────────────────────────────

// ── MOVE_FORWARD_15CM ─────────────────────────────────────────
// Drive straight forward 15 cm using encoder-based control,
// then stop.
static void test_moveForward15cm(MoveController &robot)
{
    Serial.println(F("[Test] MOVE_FORWARD_15CM — driving 15 cm"));
    robot.moveForwardCm(15);
    robot.stop();
    Serial.println(F("[Test] Done — robot stopped"));
}

static void test_moveForward90cm(MoveController &robot)
{
    Serial.println(F("[Test] MOVE_FORWARD_90CM — driving 90 cm"));
    robot.moveForwardCm(90);
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

// ── ENCODER_VERIFY ────────────────────────────────────────────
// Motors stay off. Print tick count + distance every second for
// 30 seconds. Rotate wheels by hand to verify ISR counting and
// direction. Use ACTIVE_TEST = ENCODER_VERIFY in main.cpp.
static void test_encoderVerify(MoveController &robot)
{
    Serial.println(F("[Test] ENCODER_VERIFY — rotate wheels by hand"));
    Serial.println(F("[Test] 30 seconds, printing every 1 s"));
    Serial.println(F("  s  |  L ticks  |   L cm   |  R ticks  |   R cm"));
    Serial.println(F("  ---+-----------+----------+-----------+---------"));

    robot.resetEncoders();

    const unsigned long DURATION_MS = 30000UL;
    unsigned long start     = millis();
    unsigned long lastPrint = start - 1000UL; // print immediately at t=0
    int sec = 0;

    while ((millis() - start) < DURATION_MS)
    {
        if ((millis() - lastPrint) >= 1000UL)
        {
            lastPrint += 1000UL;
            sec++;

            noInterrupts();
            long lt = robot.leftPulse;
            long rt = robot.rightPulse;
            interrupts();

            Serial.print(F("  "));
            Serial.print(sec);
            Serial.print(F("  | "));
            Serial.print(lt);
            Serial.print(F("  | "));
            Serial.print(robot.getLeftDistanceCm(), 2);
            Serial.print(F(" cm | "));
            Serial.print(rt);
            Serial.print(F("  | "));
            Serial.print(robot.getRightDistanceCm(), 2);
            Serial.println(F(" cm"));
        }
    }

    Serial.println(F("[Test] Done"));
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
    case RobotTest::MOVE_FORWARD_15CM:
        test_moveForward15cm(robot);
        break;

    case RobotTest::MOVE_FORWARD_90CM:
        test_moveForward90cm(robot);
        break;

    case RobotTest::TURN_RIGHT_90DEG:
        test_turnRight90(robot);
        break;

    case RobotTest::ENCODER_VERIFY:
        test_encoderVerify(robot);
        break;

    // ── Add new cases here as you add tests ──────────────
    }

    Serial.println(F("========================================"));
    Serial.println(F("  TEST COMPLETE — robot halted"));
    Serial.println(F("========================================\n"));
}
