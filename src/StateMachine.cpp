#include "StateMachine.h"
#include "SensorManager.h"
#include "MoveController.h"
#include "gripper_arm.h"
#include "armSlider.h"

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
void StateMachine::resetScanSequence()
{
    scanStep_      = ScanStep::SCANNING;
    scanBaseTof_   = -1;
    scanDropEncCm_ = 0.0f;
    // snapshot current encoder position as the phase start reference
    scanEncStart_  = (sensors_.getLeftEncoderCm() + sensors_.getRightEncoderCm()) * 0.5f;
}

void StateMachine::updateBoxFinding()
{
    if (phaseJustEntered_)
    {
        Serial.println(F("[BoxFind] Entered — scanning forward for boxes"));
        resetScanSequence();
        // Start driving forward at scanning speed
        robot_.setMotorSpeedsPWM(SCAN_FORWARD_PWM, SCAN_FORWARD_PWM);
    }

    // Average encoder distance travelled since phase entry
    float encNow = (sensors_.getLeftEncoderCm() + sensors_.getRightEncoderCm()) * 0.5f;
    float travelled = encNow - scanEncStart_;

    // Current left ToF (updated by SensorManager on interval)
    int leftTof = sensors_.getLeftTof();

    // ── Periodic debug print (every 1 s) ──────────────────
    unsigned long now = millis();
    if ((now - scanLastPrintMs_) >= 1000UL)
    {
        scanLastPrintMs_ = now;
        Serial.print(F("[BoxFind] tof="));
        Serial.print(leftTof);
        Serial.print(F("mm  base="));
        Serial.print(scanBaseTof_);
        Serial.print(F("mm  dist="));
        Serial.print(travelled, 1);
        Serial.print(F("cm  state="));
        Serial.println(
            scanStep_ == ScanStep::SCANNING    ? F("SCANNING") :
            scanStep_ == ScanStep::CONFIRMING  ? F("CONFIRMING") :
                                                  F("POSITIONING"));
    }

    switch (scanStep_)
    {
    // ── SCANNING: drive forward and watch for a drop ──────
    case ScanStep::SCANNING:
    {
        // Only update baseline when we have a valid, stable reading
        if (leftTof > 0)
        {
            if (scanBaseTof_ < 0)
            {
                // First valid reading — initialise baseline
                scanBaseTof_ = leftTof;
            }
            else
            {
                int drop = scanBaseTof_ - leftTof;
                if (drop >= SCAN_DROP_MM)
                {
                    // Sudden reduction detected — record encoder position and move to confirm
                    scanDropEncCm_ = encNow;
                    scanStep_ = ScanStep::CONFIRMING;
                    Serial.print(F("[BoxFind] Drop detected at tof="));
                    Serial.print(leftTof);
                    Serial.print(F("mm (base="));
                    Serial.print(scanBaseTof_);
                    Serial.println(F("mm) — confirming"));
                }
                else
                {
                    // No drop: keep updating baseline (slow drift OK, sudden jump = box)
                    // Use a gentle low-pass so baseline doesn't chase a real box
                    scanBaseTof_ = (scanBaseTof_ * 3 + leftTof) / 4;
                }
            }
        }
        break;
    }

    // ── CONFIRMING: check drop persists over SCAN_CONFIRM_CM ─
    case ScanStep::CONFIRMING:
    {
        float distSinceDrop = encNow - scanDropEncCm_;

        if (leftTof > 0)
        {
            int drop = scanBaseTof_ - leftTof;
            if (drop < SCAN_DROP_MM)
            {
                // Drop disappeared — false alarm, go back to scanning
                Serial.println(F("[BoxFind] Drop lost — false alarm, resuming scan"));
                scanBaseTof_ = leftTof;  // reset baseline to current
                scanStep_ = ScanStep::SCANNING;
                break;
            }
        }

        if (distSinceDrop >= SCAN_CONFIRM_CM)
        {
            // Box confirmed — stop, then advance SCAN_EXTRA_CM
            robot_.stop();
            Serial.println(F("[BoxFind] Box CONFIRMED — positioning"));
            // robot_.moveForwardCm((int)SCAN_EXTRA_CM);  // blocking 1 cm nudge
            scanStep_ = ScanStep::POSITIONING;
        }
        break;
    }

    // ── POSITIONING: moveForwardCm() already completed ───
    case ScanStep::POSITIONING:
    {
        Serial.print(F("[BoxFind] In position. Total travel: "));
        Serial.print(travelled);
        Serial.println(F(" cm — initiating lift"));
        transitionTo(RobotPhase::BOX_LIFTING);
        break;
    }
    }
}

// ═════════════════════════════════════════════════════════════
//  PHASE 2 — BOX LIFTING  (sub-state machine)
//
//  Slot horizontal positions (slider from home = left-most):
//    boxesLoaded == 0  →  16 cm  (first box, rightmost slot)
//    boxesLoaded == 1  →  12 cm  (second box)
//    boxesLoaded == 2  →   7 cm  (third box, closest to home)
// ═════════════════════════════════════════════════════════════

// Returns the horizontal slider target (cm) for the current box count.
static float slotPositionCm(uint8_t boxesLoaded)
{
    switch (boxesLoaded)
    {
    case 0:  return 16.0f;
    case 1:  return 12.0f;
    case 2:  return  7.0f;
    default: return  7.0f; // clamp to last slot if more boxes than expected
    }
}

void StateMachine::resetLiftSequence()
{
    liftStep_        = LiftStep::LOWER_HOOK;
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
    // All steps are blocking (servo / stepper helpers use delay internally).
    // Each step executes once per state-machine tick, then advances liftStep_.

    switch (liftStep_)
    {
    // Step 1: Lower the hook down to grab position
    case LiftStep::LOWER_HOOK:
        Serial.println(F("  [Lift] Lowering hook"));
        moveArmSmooth(currentArmDeg(), ARM_DOWN_DEG);
        delay(200); // settle
        liftStep_ = LiftStep::GRIP_BOX;
        break;

    // Step 2: Close gripper — grab the box
    case LiftStep::GRIP_BOX:
        Serial.println(F("  [Lift] Gripping box"));
        gripperClose();
        delay(500); // allow gripper to fully close
        liftStep_ = LiftStep::LIFT_VERTICAL;
        break;

    // Step 3: Lift arm back up (box held)
    case LiftStep::LIFT_VERTICAL:
        Serial.println(F("  [Lift] Lifting vertically"));
        moveArmSmooth(currentArmDeg(), ARM_UP_DEG);
        delay(200); // settle
        liftStep_ = LiftStep::MOVE_HORIZONTAL;
        break;

    // Step 4: Slide horizontally to the correct storage slot
    case LiftStep::MOVE_HORIZONTAL:
    {
        float targetCm = slotPositionCm(boxesLoaded_);
        Serial.print(F("  [Lift] Sliding to slot "));
        Serial.print(boxesLoaded_);
        Serial.print(F(" → "));
        Serial.print(targetCm, 1);
        Serial.println(F(" cm"));
        armSliderMoveTo(targetCm);
        liftStep_ = LiftStep::LOWER_PARTIAL;
        break;
    }

    // Step 5: Lower arm partially over the storage slot
    case LiftStep::LOWER_PARTIAL:
        Serial.println(F("  [Lift] Lowering partially over slot"));
        moveArmSmooth(currentArmDeg(), ARM_PARTIAL_DEG);
        delay(200); // settle
        liftStep_ = LiftStep::RELEASE_GRIP;
        break;

    // Step 6: Open gripper — deposit the box
    case LiftStep::RELEASE_GRIP:
        Serial.println(F("  [Lift] Releasing grip"));
        gripperOpen();
        delay(400); // allow gripper to fully open
        liftStep_ = LiftStep::RETRACT_HOOK;
        break;

    // Step 7: Raise arm and return slider to home
    case LiftStep::RETRACT_HOOK:
        Serial.println(F("  [Lift] Returning arm & slider to home"));
        moveArmSmooth(currentArmDeg(), ARM_UP_DEG);
        armSliderReturn(); // slider back to 0 cm
        liftStep_ = LiftStep::DONE;
        break;

    // Sequence complete — decide next phase
    case LiftStep::DONE:
    {
        boxesLoaded_++;
        Serial.print(F("  [Lift] Box stored. Total loaded: "));
        Serial.println(boxesLoaded_);

        const uint8_t TOTAL_BOXES = 3;
        if (boxesLoaded_ < TOTAL_BOXES)
        {
            transitionTo(RobotPhase::BOX_FINDING);
        }
        else
        {
            Serial.println(F("[BoxLift] All boxes collected — navigating to next section"));
            transitionTo(RobotPhase::PATH_FINDING);
        }
        break;
    }
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
