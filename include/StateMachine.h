#pragma once

#include <Arduino.h>

// Forward declarations
class SensorManager;
class MoveController;

// ─────────────────────────────────────────────────────────────
//  Robot operational phases
// ─────────────────────────────────────────────────────────────
enum class RobotPhase : uint8_t
{
    IDLE,            // Power-on default / all work complete
    BOX_FINDING,     // Phase 1 — locate boxes using encoders + ToF
    BOX_LIFTING,     // Phase 2 — pick & stack boxes (no sensors)
    PATH_FINDING,    // Phase 3 — navigate to next section
    LINE_FOLLOWING,  // Phase 4 — IR line-follow to insertion zone
    BOX_INSERTION    // Phase 5 — insert boxes into hole
};

// ─────────────────────────────────────────────────────────────
//  StateMachine — drives the robot through its phases.
//  Call update() once per loop().
// ─────────────────────────────────────────────────────────────
class StateMachine
{
public:
    StateMachine(SensorManager &sensors,
                 MoveController &robot);

    void begin();          // Set initial phase (BOX_FINDING)
    void update();         // Dispatch to current phase handler

    RobotPhase currentPhase() const { return phase_; }

    // Manually force a phase (useful for debugging / serial commands)
    void setPhase(RobotPhase newPhase);

private:
    void transitionTo(RobotPhase newPhase);

    // Per-phase lifecycle — called by update()
    void configureSensors(RobotPhase phase);

    // Phase handlers
    void updateBoxFinding();
    void updateBoxLifting();
    void updatePathFinding();
    void updateLineFollowing();
    void updateBoxInsertion();

    // ── Box-finding sub-state machine ──────────────────────
    enum class ScanStep : uint8_t
    {
        SCANNING,       // Driving forward, watching for a ToF drop
        CONFIRMING,     // Drop seen — verifying it persists over 5 cm
        POSITIONING,    // Confirmed — advancing the final 1 cm
    };

    void resetScanSequence();

    ScanStep scanStep_       = ScanStep::SCANNING;
    int      scanBaseTof_    = -1;  // Stable ToF reading before drop (mm)
    float    scanDropEncCm_  = 0.0f; // Encoder position when drop first seen (cm)
    float    scanEncStart_   = 0.0f; // Encoder position at phase entry (cm)
    unsigned long scanLastPrintMs_ = 0; // Debug print throttle

    // Detection thresholds (tune as needed)
    static constexpr int   SCAN_DROP_MM       = 70;   // 7 cm drop triggers detection
    static constexpr float SCAN_CONFIRM_CM    = 8.0f; // hold drop for 8 cm to confirm
    static constexpr float SCAN_EXTRA_CM      = 1.0f; // advance 1 cm after confirmation
    static constexpr int   SCAN_FORWARD_PWM   = 50;

    // ── Box-lifting sub-state machine ──────────────────────
    enum class LiftStep : uint8_t
    {
        LOWER_HOOK,
        GRIP_BOX,
        LIFT_VERTICAL,
        MOVE_HORIZONTAL,
        LOWER_PARTIAL,
        RELEASE_GRIP,
        RETRACT_HOOK,
        DONE
    };

    void resetLiftSequence();

    LiftStep liftStep_       = LiftStep::LOWER_HOOK;
    uint8_t  boxesLoaded_    = 0;
    unsigned long liftStepStartMs_ = 0;

    // ── Core references ────────────────────────────────────
    SensorManager  &sensors_;
    MoveController &robot_;

    RobotPhase phase_ = RobotPhase::IDLE;
    bool       phaseJustEntered_ = true;  // true on first update() after transition
};
