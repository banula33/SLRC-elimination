#include "ToFArray.h"

ToFArray::ToFArray(
  uint8_t leftXshutPin,
  uint8_t rightXshutPin,
  uint8_t frontXshutPin,
  int16_t leftOffsetMm,
  int16_t rightOffsetMm,
  int16_t frontOffsetMm
) : leftXshutPin_(leftXshutPin),
    rightXshutPin_(rightXshutPin),
    frontXshutPin_(frontXshutPin),
    leftOffsetMm_(leftOffsetMm),
    rightOffsetMm_(rightOffsetMm),
    frontOffsetMm_(frontOffsetMm) {}

bool ToFArray::begin() {
  Wire.begin();

  pinMode(leftXshutPin_, OUTPUT);
  pinMode(rightXshutPin_, OUTPUT);
  pinMode(frontXshutPin_, OUTPUT);

  shutdownAll();
  delay(10);

  if (!initSensor(leftXshutPin_)) {
    return false;
  }

  if (!initSensor(rightXshutPin_)) {
    return false;
  }

  if (!initSensor(frontXshutPin_)) {
    return false;
  }

  return true;
}

int ToFArray::readLeftMm() {
  return applyOffsetMm(readDistanceMm(leftXshutPin_), leftOffsetMm_);
}

int ToFArray::readRightMm() {
  return applyOffsetMm(readDistanceMm(rightXshutPin_), rightOffsetMm_);
}

int ToFArray::readFrontMm() {
  return applyOffsetMm(readDistanceMm(frontXshutPin_), frontOffsetMm_);
}

ToFArray::Readings ToFArray::readAllMm() {
  Readings result;
  result.leftMm = readLeftMm();
  result.rightMm = readRightMm();
  result.frontMm = readFrontMm();
  return result;
}

void ToFArray::setOffsetsMm(int16_t leftOffsetMm, int16_t rightOffsetMm, int16_t frontOffsetMm) {
  leftOffsetMm_ = leftOffsetMm;
  rightOffsetMm_ = rightOffsetMm;
  frontOffsetMm_ = frontOffsetMm;
}

int ToFArray::readDistanceMm(uint8_t xshutPin) {
  shutdownAll();
  digitalWrite(xshutPin, HIGH);
  delay(60);

  if (!sensor_.begin(0x29, false, &Wire)) {
    delay(60);
    if (!sensor_.begin(0x29, false, &Wire)) {
      shutdownAll();
      return -1;
    }
  }

  sensor_.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  delay(50);

  VL53L0X_RangingMeasurementData_t measure;
  sensor_.rangingTest(&measure, false);
  if (measure.RangeStatus != 0) {
    delay(20);
    sensor_.rangingTest(&measure, false);
  }

  shutdownAll();

  if (measure.RangeStatus != 0) {
    return -1;
  }

  return static_cast<int>(measure.RangeMilliMeter);
}

int ToFArray::applyOffsetMm(int distanceMm, int16_t offsetMm) {
  if (distanceMm < 0) {
    return -1;
  }

  int adjusted = distanceMm - static_cast<int>(offsetMm);
  if (adjusted < 0) {
    return 0;
  }

  return adjusted;
}

void ToFArray::shutdownAll() {
  digitalWrite(leftXshutPin_, LOW);
  digitalWrite(rightXshutPin_, LOW);
  digitalWrite(frontXshutPin_, LOW);
}

bool ToFArray::initSensor(uint8_t xshutPin) {
  shutdownAll();
  digitalWrite(xshutPin, HIGH);
  delay(60);

  if (!sensor_.begin(0x29, false, &Wire)) {
    delay(60);
    if (!sensor_.begin(0x29, false, &Wire)) {
      shutdownAll();
      return false;
    }
  }

  sensor_.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  delay(50);
  shutdownAll();
  return true;
}
