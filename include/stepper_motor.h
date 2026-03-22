#pragma once

#include <Arduino.h>

constexpr uint8_t IN1 = 32;
constexpr uint8_t IN2 = 33;
constexpr uint8_t IN3 = 34;
constexpr uint8_t IN4 = 35;

extern int stepDelay;

void initStepper();
void rotateSteps(long steps);
