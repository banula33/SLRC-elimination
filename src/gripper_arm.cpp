#include "gripper_arm.h"
#include <Servo.h>

static Servo armServo;
static Servo gripperServo;

void initGripperArm(uint8_t armPin, uint8_t gripperPin) {
  armServo.attach(armPin);
  gripperServo.attach(gripperPin);

  armServo.write(0);
  gripperServo.write(180);
  delay(1000);
}

void moveArmSmooth(int fromDeg, int toDeg, int dlyMs) {
  fromDeg = constrain(fromDeg, 0, 180);
  toDeg = constrain(toDeg, 0, 180);

  if (fromDeg <= toDeg) {
    for (int p = fromDeg; p <= toDeg; p++) {
      armServo.write(p);
      delay(dlyMs);
    }
  } else {
    for (int p = fromDeg; p >= toDeg; p--) {
      armServo.write(p);
      delay(dlyMs);
    }
  }
}

void gripperOpen() {
  gripperServo.write(180);
}

void gripperClose() {
  gripperServo.write(0);
}