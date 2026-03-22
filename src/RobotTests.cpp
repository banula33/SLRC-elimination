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
// In-place right turn by exactly 90° using encoder-based PID.
static void test_turnRight90(MoveController &robot)
{
    Serial.println(F("[Test] TURN_RIGHT_90DEG — encoder PID turn"));
    robot.turnRightDeg(85);
    robot.stop();
    Serial.println(F("[Test] Done — turned right 90°"));
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
