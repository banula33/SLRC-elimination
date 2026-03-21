#include "stepper_motor.h"

int stepDelay = 2;

static void stepMotor(int step) {
  switch (step % 4) {
    case 0:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    case 1:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    case 2:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      break;
    case 3:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break;
  }
}

void initStepper() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

// Positive = forward, Negative = backward
void rotateSteps(long steps) {
  if (steps == 0) return;

  long count = (steps > 0) ? steps : -steps;
  for (long i = 0; i < count; i++) {
    int phase = (steps > 0) ? (i % 4) : (3 - (i % 4));
    stepMotor(phase);
    delay(stepDelay);
  }
}
