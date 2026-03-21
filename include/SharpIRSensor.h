#pragma once
#include <Arduino.h>

class SharpIRSensor {
private:
  int sensorPin;
  float calibrationOffset;

public:
  SharpIRSensor(int pin = A0);
  
  void begin();
  float readDistance();
  void setCalibrationOffset(float offset);
};