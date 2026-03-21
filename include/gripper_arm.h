#pragma once
#include <Arduino.h>

void initGripperArm(uint8_t armPin = 39, uint8_t gripperPin = 41);
void moveArmSmooth(int fromDeg, int toDeg, int dlyMs = 15);
void gripperOpen();
void gripperClose();