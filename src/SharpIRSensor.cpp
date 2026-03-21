#include "SharpIRSensor.h"

SharpIRSensor::SharpIRSensor(int pin) 
  : sensorPin(pin), calibrationOffset(0.0f) {}

void SharpIRSensor::begin() {
  pinMode(sensorPin, INPUT);
}

float SharpIRSensor::readDistance() {
  // Take multiple samples for stability
  int samples = 10;
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(sensorPin);
  }
  int rawValue = sum / samples;
  
  float voltage = (rawValue / 1023.0) * 5.0;
  
  // Lookup table calibration (adjust these values based on YOUR sensor)
  // Format: {voltage, distance_cm}
  if (voltage > 4.5) return 4.0;
  if (voltage > 3.5) return 5.0;
  if (voltage > 3.0) return 6.0;
  if (voltage > 2.5) return 7.0;
  if (voltage > 2.2) return 8.0;
  if (voltage > 2.0) return 9.0;
  if (voltage > 1.8) return 10.0;
  if (voltage > 1.5) return 12.0;
  if (voltage > 1.3) return 15.0;
  if (voltage > 1.1) return 20.0;
  if (voltage > 0.9) return 25.0;
  if (voltage > 0.7) return 30.0;
  if (voltage > 0.5) return 40.0;
  
  return 80.0; // Out of range
}

void SharpIRSensor::setCalibrationOffset(float offset) {
  calibrationOffset = offset;
}