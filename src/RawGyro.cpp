#include "RawGyro.h"
#include <Wire.h>

static const int MPU_ADDR = 0x68;

void setupRawGyro()
{
  Wire.begin();
  Wire.setClock(400000);

  // Wake up MPU-6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Set to 0 to wake up
  Wire.endTransmission(true);

  // Optional: Set accelerometer range to +/-2g (default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00); // +/-2g
  Wire.endTransmission(true);

  // Optional: Set gyroscope range to +/-250 deg/s (default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00); // +/-250 deg/s
  Wire.endTransmission(true);
}

bool updateRawGyro(RawGyroReadings &outReadings)
{
  // Request data starting from register 0x3B (ACCEL_XOUT_H)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  // Request 14 bytes
  uint8_t bytesRead = Wire.requestFrom(MPU_ADDR, 14, true);
  if (bytesRead != 14)
  {
    return false;
  }

  // Read accelerometer (each value is 2 bytes, high byte first)
  outReadings.accelX = (Wire.read() << 8) | Wire.read();
  outReadings.accelY = (Wire.read() << 8) | Wire.read();
  outReadings.accelZ = (Wire.read() << 8) | Wire.read();

  // Read temperature
  outReadings.temperature = (Wire.read() << 8) | Wire.read();

  // Read gyroscope
  outReadings.gyroX = (Wire.read() << 8) | Wire.read();
  outReadings.gyroY = (Wire.read() << 8) | Wire.read();
  outReadings.gyroZ = (Wire.read() << 8) | Wire.read();

  return true;
}

float rawGyroTemperatureC(int16_t temperatureRaw)
{
  // Temp (deg C) = (raw / 340.0) + 36.53
  return (temperatureRaw / 340.0f) + 36.53f;
}
