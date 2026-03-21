#include "gripper_arm.h"
#include <Servo.h>

// ─────────────────────────────────────────────────────────────
//  Module-level servo objects
// ─────────────────────────────────────────────────────────────
static Servo armServo;
static Servo gripperServo;
static int   lastArmDeg = ARM_UP_DEG;

// ─────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────
void initGripperArm(uint8_t armPin, uint8_t gripperPin)
{
    armServo.attach(armPin);
    gripperServo.attach(gripperPin);

    // Start at known home state
    armServo.write(ARM_UP_DEG);
    gripperServo.write(GRIPPER_OPEN_DEG);
    lastArmDeg = ARM_UP_DEG;
    delay(500);
}

void moveArmSmooth(int fromDeg, int toDeg, int dlyMs)
{
    fromDeg = constrain(fromDeg, 0, 180);
    toDeg   = constrain(toDeg,   0, 180);

    int step = (fromDeg <= toDeg) ? 1 : -1;
    for (int p = fromDeg; p != toDeg + step; p += step)
    {
        armServo.write(p);
        delay(dlyMs);
    }
    lastArmDeg = toDeg;
}

void gripperOpen()
{
    gripperServo.write(GRIPPER_OPEN_DEG);
}

void gripperClose()
{
    gripperServo.write(GRIPPER_CLOSE_DEG);
}

int currentArmDeg()
{
    return lastArmDeg;
}