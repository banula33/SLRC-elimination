#pragma once

#include <Arduino.h>

struct RawGyroReadings
{
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  int16_t temperature;
};

void setupRawGyro();
bool updateRawGyro(RawGyroReadings &outReadings);
float rawGyroTemperatureC(int16_t temperatureRaw);
