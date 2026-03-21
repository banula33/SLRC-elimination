#pragma once

#include <Arduino.h>

constexpr uint8_t IN1 = 8;
constexpr uint8_t IN2 = 9;
constexpr uint8_t IN3 = 10;
constexpr uint8_t IN4 = 11;

extern int stepDelay;

void initStepper();
void rotateSteps(long steps);
