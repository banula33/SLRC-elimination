
#include <Arduino.h>
#include "MoveController.h"
#include "ToFArray.h"
#include "SensorManager.h"
#include "StateMachine.h"
#include "RawGyro.h"
#include "gripper_arm.h"
#include "armSlider.h"
#include "RobotTests.h"

// ─────────────────────────────────────────────────────────────
//  ★  TESTING MODE FLAG
//     Set to true  → run 10-second sensor diagnostic on startup,
//     Set to false → skip directly to normal state-machine operation.
// ─────────────────────────────────────────────────────────────
static constexpr bool TESTING_MODE = true;

// ─────────────────────────────────────────────────────────────
//  ★  ROBOT TEST FLAG
//     Set RUN_ROBOT_TEST to true to run a specific movement or
//     mechanism test instead of the normal state machine.
//     Choose which test to run by setting ACTIVE_TEST.
//     When done, set RUN_ROBOT_TEST back to false.
// ─────────────────────────────────────────────────────────────
static constexpr bool RUN_ROBOT_TEST = false;
static constexpr RobotTest ACTIVE_TEST = RobotTest::MOVE_FORWARD_90CM;

// ─────────────────────────────────────────────────────────────
//  Hardware instances
// ─────────────────────────────────────────────────────────────
// Motor driver: ENA=4, IN1=6, IN2=7, ENB=5, IN3=8, IN4=9
// Wheel ø 6.5 cm, 20 PPR, 15 cm wheelbase, max 255, min 50
MoveController robot(4, 7, 6, 5, 9, 8, 6.5, 1630, 18.0, 255, 50);

// ToF XSHUT pins
const uint8_t TOF_LEFT_XSHUT_PIN = 26;
const uint8_t TOF_RIGHT_XSHUT_PIN = 22;
const uint8_t TOF_FRONT_XSHUT_PIN = 24;

ToFArray tofArray(TOF_LEFT_XSHUT_PIN, TOF_RIGHT_XSHUT_PIN, TOF_FRONT_XSHUT_PIN);

// Nemma motor driver pins
const uint8_t NEMMA_MOTOR_ENABLE = 23;
const uint8_t NEMMA_MOTOR_STEP = 10;
const uint8_t NEMMA_MOTOR_DIR = 11;

// ─────────────────────────────────────────────────────────────
//  Core managers
// ─────────────────────────────────────────────────────────────
SensorManager sensorMgr(robot, tofArray);
StateMachine stateMachine(sensorMgr, robot);

// ─────────────────────────────────────────────────────────────
//  Sensor diagnostic routine (runs only when TESTING_MODE = true)
//  Collects one reading per sensor every 500 ms for 10 seconds,
//  then prints them all to Serial.
// ─────────────────────────────────────────────────────────────
static void runSensorTests()
{
    Serial.println(F("\n========================================"));
    Serial.println(F("  SENSOR TEST MODE — 10 seconds"));
    Serial.println(F("========================================"));

    const unsigned long TEST_DURATION_MS = 5000UL;
    const unsigned long SAMPLE_INTERVAL_MS = 500UL;

    unsigned long testStart = millis();
    unsigned long lastSample = testStart - SAMPLE_INTERVAL_MS; // take first sample immediately
    int sampleNum = 0;

    while ((millis() - testStart) < TEST_DURATION_MS)
    {
        unsigned long now = millis();
        if ((now - lastSample) >= SAMPLE_INTERVAL_MS)
        {
            lastSample = now;
            sampleNum++;

            Serial.print(F("\n--- Sample #"));
            Serial.print(sampleNum);
            Serial.print(F("  (t="));
            Serial.print((now - testStart) / 1000.0f, 1);
            Serial.println(F(" s) ---"));

            // ── ToF sensors ────────────────────────────────
            Serial.print(F("  ToF Left  : "));
            int tl = tofArray.readLeftMm();
            if (tl < 0)
                Serial.println(F("ERR"));
            else
            {
                Serial.print(tl);
                Serial.println(F(" mm"));
            }

            Serial.print(F("  ToF Front : "));
            int tf = tofArray.readFrontMm();
            if (tf < 0)
                Serial.println(F("ERR"));
            else
            {
                Serial.print(tf);
                Serial.println(F(" mm"));
            }

            Serial.print(F("  ToF Right : "));
            int tr = tofArray.readRightMm();
            if (tr < 0)
                Serial.println(F("ERR"));
            else
            {
                Serial.print(tr);
                Serial.println(F(" mm"));
            }

            // ── Encoders (ISR-driven — always up to date) ────
            Serial.print(F("  Enc Left  : "));
            Serial.print(robot.getLeftDistanceCm(), 2);
            Serial.println(F(" cm"));

            Serial.print(F("  Enc Right : "));
            Serial.print(robot.getRightDistanceCm(), 2);
            Serial.println(F(" cm"));

            // ── Gyro ───────────────────────────────────────
            RawGyroReadings g;
            if (updateRawGyro(g))
            {
                Serial.print(F("  Gyro  aX="));
                Serial.print(g.accelX);
                Serial.print(F("  aY="));
                Serial.print(g.accelY);
                Serial.print(F("  aZ="));
                Serial.print(g.accelZ);
                Serial.print(F("  gX="));
                Serial.print(g.gyroX);
                Serial.print(F("  gY="));
                Serial.print(g.gyroY);
                Serial.print(F("  gZ="));
                Serial.print(g.gyroZ);
                Serial.print(F("  temp="));
                Serial.print(rawGyroTemperatureC(g.temperature), 1);
                Serial.println(F(" C"));
            }
            else
            {
                Serial.println(F("  Gyro  : ERR (not ready)"));
            }
        }
    }

    Serial.println(F("\n========================================"));
    Serial.println(F("  SENSOR TEST COMPLETE — starting robot"));
    Serial.println(F("========================================\n"));
}

// ─────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────
void setup()
{
    pinMode(NEMMA_MOTOR_ENABLE, OUTPUT);
    pinMode(NEMMA_MOTOR_STEP, OUTPUT);
    pinMode(NEMMA_MOTOR_DIR, OUTPUT);
    digitalWrite(NEMMA_MOTOR_ENABLE, HIGH);

    Serial.begin(115200);

    // Motor drivers
    robot.begin();

    // Encoders: rightA=2, rightB=3, leftA=18, leftB=19
    robot.configureEncoders(2, 3, 18, 19);

    // Initialise sensors
    sensorMgr.begin();

    // Initialise gripper arm servos (arm pin 39, gripper pin 41)
    initGripperArm();

    // Initialise horizontal slider (pins already configured above)
    initArmSlider();

    if (TESTING_MODE)
    {
        runSensorTests();
    }

    // Start the state machine at Phase 1 (BOX_FINDING)
    // Skip if a robot test is selected — halt after test instead.
    if (RUN_ROBOT_TEST)
    {
        runRobotTest(ACTIVE_TEST, robot);
        while (true)
        { /* halt — re-flash to resume normal operation */
        }
    }

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
