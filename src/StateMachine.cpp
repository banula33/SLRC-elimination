#include "StateMachine.h"
#include "SensorManager.h"
#include "MoveController.h"
#include "gripper_arm.h"
#include "stepper_motor.h"

// ─────────────────────────────────────────────────────────────
//  Construction
// ─────────────────────────────────────────────────────────────
StateMachine::StateMachine(SensorManager &sensors,
                           MoveController &robot)
    : sensors_(sensors), robot_(robot) {}

// ─────────────────────────────────────────────────────────────
//  begin() — called once from setup()
// ─────────────────────────────────────────────────────────────
void StateMachine::begin()
{
    transitionTo(RobotPhase::BOX_FINDING);
}

// ─────────────────────────────────────────────────────────────
//  update() — called every loop()
// ─────────────────────────────────────────────────────────────
void StateMachine::update()
{
    // Let SensorManager poll only the currently-enabled sensors
    sensors_.update();

    switch (phase_)
    {
    case RobotPhase::IDLE:           break; // nothing to do
    case RobotPhase::BOX_FINDING:    updateBoxFinding();    break;
    case RobotPhase::BOX_LIFTING:    updateBoxLifting();    break;
    case RobotPhase::PATH_FINDING:   updatePathFinding();   break;
    case RobotPhase::LINE_FOLLOWING: updateLineFollowing(); break;
    case RobotPhase::BOX_INSERTION:  updateBoxInsertion();  break;
    }

    phaseJustEntered_ = false;
}

// ─────────────────────────────────────────────────────────────
//  Phase transitions
// ─────────────────────────────────────────────────────────────
void StateMachine::setPhase(RobotPhase newPhase)
{
    transitionTo(newPhase);
}

void StateMachine::transitionTo(RobotPhase newPhase)
{
    Serial.print(F("[SM] Phase "));
    Serial.print((uint8_t)phase_);
    Serial.print(F(" -> "));
    Serial.println((uint8_t)newPhase);

    phase_ = newPhase;
    phaseJustEntered_ = true;
    configureSensors(newPhase);
}

// ─────────────────────────────────────────────────────────────
//  Sensor configuration per phase
// ─────────────────────────────────────────────────────────────
void StateMachine::configureSensors(RobotPhase phase)
{
    sensors_.disableAll();

    switch (phase)
    {
    case RobotPhase::BOX_FINDING:
        sensors_.enableEncoders(true);
        sensors_.enableTofLeft(true);
        sensors_.enableTofFront(true);
        break;

    case RobotPhase::BOX_LIFTING:
        // No sensors needed — pure actuator sequence
        break;

    case RobotPhase::PATH_FINDING:
        sensors_.enableEncoders(true);
        sensors_.enableTofLeft(true);
        sensors_.enableTofFront(true);
        break;

    case RobotPhase::LINE_FOLLOWING:
        sensors_.enableIrArray(true);
        sensors_.enableTofFront(true);
        break;

    case RobotPhase::BOX_INSERTION:
        // Will be configured when implemented
        break;

    case RobotPhase::IDLE:
        break;
    }
}

// ═════════════════════════════════════════════════════════════
//  PHASE 1 — BOX FINDING
// ═════════════════════════════════════════════════════════════
void StateMachine::updateBoxFinding()
{
    if (phaseJustEntered_)
    {
        Serial.println(F("[BoxFind] Entered — scanning for boxes"));
        // TODO: reset encoder baseline, start scanning pattern
    }

    // ── Read available sensors ────────────────────────────
    // int leftTof  = sensors_.getLeftTof();
    // int frontTof = sensors_.getFrontTof();
    // float leftCm = sensors_.getLeftEncoderCm();

    // TODO: Implement box-detection logic
    //       When a box is detected and robot is in position:
    //       transitionTo(RobotPhase::BOX_LIFTING);
}

// ═════════════════════════════════════════════════════════════
//  PHASE 2 — BOX LIFTING  (sub-state machine)
// ═════════════════════════════════════════════════════════════
void StateMachine::resetLiftSequence()
{
    liftStep_ = LiftStep::LOWER_HOOK;
    liftStepStartMs_ = millis();
}

void StateMachine::updateBoxLifting()
{
    if (phaseJustEntered_)
    {
        Serial.print(F("[BoxLift] Entered — box #"));
        Serial.println(boxesLoaded_ + 1);
        resetLiftSequence();
    }

    // ── Sub-state machine ─────────────────────────────────
    // Each step is blocking-style using the existing servo/stepper
    // helpers which already use delay() internally.
    //
    // For a non-blocking approach later, track elapsed time
    // against liftStepStartMs_ and advance liftStep_ when done.

    switch (liftStep_)
    {
    case LiftStep::LOWER_HOOK:
        Serial.println(F("  [Lift] Lowering hook"));
        // TODO: moveArmSmooth(currentArmPos, HOOK_DOWN_DEG);
        liftStep_ = LiftStep::GRIP_BOX;
        liftStepStartMs_ = millis();
        break;

    case LiftStep::GRIP_BOX:
        Serial.println(F("  [Lift] Gripping box"));
        // TODO: gripperClose();
        liftStep_ = LiftStep::LIFT_VERTICAL;
        liftStepStartMs_ = millis();
        break;

    case LiftStep::LIFT_VERTICAL:
        Serial.println(F("  [Lift] Lifting vertically"));
        // TODO: moveArmSmooth(HOOK_DOWN_DEG, HOOK_UP_DEG);
        liftStep_ = LiftStep::MOVE_HORIZONTAL;
        liftStepStartMs_ = millis();
        break;

    case LiftStep::MOVE_HORIZONTAL:
    {
        // Horizontal offset depends on how many boxes already loaded
        Serial.print(F("  [Lift] Moving horizontal — slot "));
        Serial.println(boxesLoaded_);
        // TODO: rotateSteps(STEPS_PER_SLOT * boxesLoaded_);
        liftStep_ = LiftStep::LOWER_PARTIAL;
        liftStepStartMs_ = millis();
        break;
    }

    case LiftStep::LOWER_PARTIAL:
        Serial.println(F("  [Lift] Lowering partially"));
        // TODO: moveArmSmooth(HOOK_UP_DEG, PARTIAL_DOWN_DEG);
        liftStep_ = LiftStep::RELEASE_GRIP;
        liftStepStartMs_ = millis();
        break;

    case LiftStep::RELEASE_GRIP:
        Serial.println(F("  [Lift] Releasing grip"));
        // TODO: gripperOpen();
        liftStep_ = LiftStep::RETRACT_HOOK;
        liftStepStartMs_ = millis();
        break;

    case LiftStep::RETRACT_HOOK:
        Serial.println(F("  [Lift] Retracting hook to start"));
        // TODO: moveArmSmooth(PARTIAL_DOWN_DEG, HOOK_UP_DEG);
        //       + move horizontal back to leftmost position
        liftStep_ = LiftStep::DONE;
        break;

    case LiftStep::DONE:
        boxesLoaded_++;
        Serial.print(F("  [Lift] Box loaded. Total: "));
        Serial.println(boxesLoaded_);

        // Decide: more boxes to collect, or move on?
        // TODO: replace constant with actual target count
        const uint8_t TOTAL_BOXES = 3;
        if (boxesLoaded_ < TOTAL_BOXES)
        {
            transitionTo(RobotPhase::BOX_FINDING);
        }
        else
        {
            Serial.println(F("[BoxLift] All boxes collected"));
            transitionTo(RobotPhase::PATH_FINDING);
        }
        break;
    }
}

// ═════════════════════════════════════════════════════════════
//  PHASE 3 — PATH FINDING  (stub)
// ═════════════════════════════════════════════════════════════
void StateMachine::updatePathFinding()
{
    if (phaseJustEntered_)
    {
        Serial.println(F("[PathFind] Entered — navigation logic TBD"));
    }

    // TODO: Implement navigation to next section.
    //       Uses encoders + ToF sensors.
    //       When destination reached:
    //       transitionTo(RobotPhase::LINE_FOLLOWING);
}

// ═════════════════════════════════════════════════════════════
//  PHASE 4 — LINE FOLLOWING
// ═════════════════════════════════════════════════════════════
void StateMachine::updateLineFollowing()
{
    if (phaseJustEntered_)
    {
        Serial.println(F("[LineFol] Entered — following line to insertion zone"));
    }

    // ── Read sensors ──────────────────────────────────────
    // const uint8_t *ir = sensors_.getIrArray();
    // int frontTof       = sensors_.getFrontTof();

    // TODO: PID line-following using IR array
    //       Stop condition: frontTof < THRESHOLD_MM
    //       When arrived:
    //       transitionTo(RobotPhase::BOX_INSERTION);
}

// ═════════════════════════════════════════════════════════════
//  PHASE 5 — BOX INSERTION  (stub)
// ═════════════════════════════════════════════════════════════
void StateMachine::updateBoxInsertion()
{
    if (phaseJustEntered_)
    {
        Serial.println(F("[BoxIns] Entered — insertion mechanism TBD"));
    }

    // TODO: Implement box insertion using separate hook mechanism.
    //       When finished:
    //       transitionTo(RobotPhase::IDLE);
}
