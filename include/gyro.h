#pragma once

#include <Arduino.h>

extern bool gyro_ok;
extern float currentYaw;
extern float currentPitch;

void setupGyro();
bool updateGyro();
void flushGyro();
float wrap360(float angleDeg);
