#ifndef TOF_ARRAY_H
#define TOF_ARRAY_H

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────
//  ToFArray — three VL53L0X sensors, each with a unique I2C
//  address assigned at init. Sensors stay powered on;
//  reads are fast (~33 ms per sensor, no re-init overhead).
// ─────────────────────────────────────────────────────────────

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

  // Assign unique I2C addresses and keep all sensors active.
  bool begin();

  // Fast reads — no shutdown/re-init, just rangingTest().
  int readLeftMm();
  int readRightMm();
  int readFrontMm();
  Readings readAllMm();

  void setOffsetsMm(int16_t leftOffsetMm, int16_t rightOffsetMm, int16_t frontOffsetMm);

private:
  int applyOffsetMm(int distanceMm, int16_t offsetMm);

  uint8_t leftXshutPin_;
  uint8_t rightXshutPin_;
  uint8_t frontXshutPin_;

  int16_t leftOffsetMm_;
  int16_t rightOffsetMm_;
  int16_t frontOffsetMm_;

  // One sensor object per physical sensor (each has a unique address)
  Adafruit_VL53L0X sensorLeft_;
  Adafruit_VL53L0X sensorRight_;
  Adafruit_VL53L0X sensorFront_;

  bool leftOk_  = false;
  bool rightOk_ = false;
  bool frontOk_ = false;

  // Unique I2C addresses for each sensor
  static constexpr uint8_t ADDR_LEFT  = 0x30;
  static constexpr uint8_t ADDR_RIGHT = 0x31;
  static constexpr uint8_t ADDR_FRONT = 0x32;
};

#endif // TOF_ARRAY_H
