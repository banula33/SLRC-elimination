#pragma once

#include "MoveController.h"

// ─────────────────────────────────────────────────────────────
//  Robot test suite — on-demand movement & mechanism tests.
//
//  Usage in main.cpp:
//    Set ACTIVE_TEST to the test you want, then set
//    RUN_ROBOT_TEST = true.  The test runs once in setup()
//    and the robot halts afterwards (does not enter the
//    normal state machine).
//
//  Add new tests as plain functions here, then register them
//  in runRobotTest() in RobotTests.cpp.
// ─────────────────────────────────────────────────────────────

// ── Available tests ───────────────────────────────────────────
enum class RobotTest : uint8_t
{
    MOVE_FORWARD_10CM,     // Drive straight 10 cm and stop
    TURN_RIGHT_90DEG,      // In-place right turn 90°

    // ── Add new tests below ──────────────────────────────────
    // MOVE_BACKWARD_10CM,
    // TURN_LEFT_90DEG,
    // GRIPPER_OPEN_CLOSE,
    // SLIDER_FULL_SWEEP,
};

// Call from setup() — runs the selected test once then returns.
void runRobotTest(RobotTest test, MoveController &robot);
