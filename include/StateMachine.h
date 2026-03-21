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
