#ifndef TOF_ARRAY_H
#define TOF_ARRAY_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>

class ToFArray {
public:
  struct Readings {
    int leftMm;
    int rightMm;
    int frontMm;
  };

  ToFArray(
    uint8_t leftXshutPin,
    uint8_t rightXshutPin,
    uint8_t frontXshutPin,
    int16_t leftOffsetMm = 20,
    int16_t rightOffsetMm = 40,
    int16_t frontOffsetMm = 25
  );

  bool begin();

  int readLeftMm();
  int readRightMm();
  int readFrontMm();
  Readings readAllMm();

  void setOffsetsMm(int16_t leftOffsetMm, int16_t rightOffsetMm, int16_t frontOffsetMm);

private:
  int readDistanceMm(uint8_t xshutPin);
  int applyOffsetMm(int distanceMm, int16_t offsetMm);

  void shutdownAll();
  bool initSensor(uint8_t xshutPin);

  uint8_t leftXshutPin_;
  uint8_t rightXshutPin_;
  uint8_t frontXshutPin_;

  int16_t leftOffsetMm_;
  int16_t rightOffsetMm_;
  int16_t frontOffsetMm_;

  Adafruit_VL53L0X sensor_;
};

#endif // TOF_ARRAY_H
