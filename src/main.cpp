
#include <Arduino.h>
#include "MoveController.h"
#include "ToFArray.h"
#include "SensorManager.h"
#include "StateMachine.h"

// ─────────────────────────────────────────────────────────────
//  Hardware instances
// ─────────────────────────────────────────────────────────────
// Motor driver: ENA=4, IN1=6, IN2=7, ENB=5, IN3=8, IN4=9
// Wheel ø 6.5 cm, 20 PPR, 15 cm wheelbase, max 255, min 50
MoveController robot(4, 6, 7, 5, 8, 9, 6.5, 20, 15.0, 255, 50);

// ToF XSHUT pins
const uint8_t TOF_LEFT_XSHUT_PIN  = 26;
const uint8_t TOF_RIGHT_XSHUT_PIN = 22;
const uint8_t TOF_FRONT_XSHUT_PIN = 24;

ToFArray tofArray(TOF_LEFT_XSHUT_PIN, TOF_RIGHT_XSHUT_PIN, TOF_FRONT_XSHUT_PIN);

// ─────────────────────────────────────────────────────────────
//  Core managers
// ─────────────────────────────────────────────────────────────
SensorManager sensorMgr(robot, tofArray);
StateMachine  stateMachine(sensorMgr, robot);

// ─────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(500000);

    // Motor drivers
    robot.begin();

    // Encoders: rightA=2, rightB=3, leftA=18, leftB=19
    robot.configureEncoders(2, 3, 18, 19);

    // Initialise all sensors (gyro + ToF) — they start disabled
    sensorMgr.begin();

    // Start the state machine at Phase 1 (BOX_FINDING)
    stateMachine.begin();

    Serial.println(F("=== SLRC Elimination Robot Ready ==="));
}

// ─────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────
void loop()
{
    // Run the state machine (polls only enabled sensors, then runs phase logic)
    stateMachine.update();
}
