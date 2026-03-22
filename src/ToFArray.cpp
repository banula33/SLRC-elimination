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

// ─────────────────────────────────────────────────────────────
//  begin() — Assign a unique I2C address to each sensor.
//
//  The VL53L0X boots at default address 0x29. We bring up
//  sensors one at a time using XSHUT pins, re-address each
//  to a unique address, then leave them all on.
//
//  Result: all 3 sensors are alive with different addresses,
//          so reads are a simple rangingTest() — no shutdown/
//          wake/re-init overhead.
// ─────────────────────────────────────────────────────────────
bool ToFArray::begin()
{
    // Configure XSHUT pins
    pinMode(leftXshutPin_,  OUTPUT);
    pinMode(rightXshutPin_, OUTPUT);
    pinMode(frontXshutPin_, OUTPUT);

    // Shut down all 3 sensors
    digitalWrite(leftXshutPin_,  LOW);
    digitalWrite(rightXshutPin_, LOW);
    digitalWrite(frontXshutPin_, LOW);
    delay(10);

    // ── Bring up LEFT sensor and assign ADDR_LEFT ──────────
    digitalWrite(leftXshutPin_, HIGH);
    delay(10);
    if (sensorLeft_.begin(ADDR_LEFT, false, &Wire))
    {
        sensorLeft_.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
        leftOk_ = true;
        Serial.print(F("ToF Left  @ 0x"));
        Serial.println(ADDR_LEFT, HEX);
    }
    else
    {
        Serial.println(F("ToF Left  FAIL"));
    }

    // ── Bring up RIGHT sensor and assign ADDR_RIGHT ────────
    digitalWrite(rightXshutPin_, HIGH);
    delay(10);
    if (sensorRight_.begin(ADDR_RIGHT, false, &Wire))
    {
        sensorRight_.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
        rightOk_ = true;
        Serial.print(F("ToF Right @ 0x"));
        Serial.println(ADDR_RIGHT, HEX);
    }
    else
    {
        Serial.println(F("ToF Right FAIL"));
    }

    // ── Bring up FRONT sensor and assign ADDR_FRONT ────────
    digitalWrite(frontXshutPin_, HIGH);
    delay(10);
    if (sensorFront_.begin(ADDR_FRONT, false, &Wire))
    {
        sensorFront_.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
        frontOk_ = true;
        Serial.print(F("ToF Front @ 0x"));
        Serial.println(ADDR_FRONT, HEX);
    }
    else
    {
        Serial.println(F("ToF Front FAIL"));
    }

    return leftOk_ || rightOk_ || frontOk_;
}

// ─────────────────────────────────────────────────────────────
//  Fast reads — sensor is already on with a unique address.
//  Just do rangingTest(), ~33ms per read.
// ─────────────────────────────────────────────────────────────
int ToFArray::readLeftMm()
{
    if (!leftOk_) return -1;

    VL53L0X_RangingMeasurementData_t m;
    sensorLeft_.rangingTest(&m, false);

    if (m.RangeStatus != 0) return -1;
    return applyOffsetMm((int)m.RangeMilliMeter, leftOffsetMm_);
}

int ToFArray::readRightMm()
{
    if (!rightOk_) return -1;

    VL53L0X_RangingMeasurementData_t m;
    sensorRight_.rangingTest(&m, false);

    if (m.RangeStatus != 0) return -1;
    return applyOffsetMm((int)m.RangeMilliMeter, rightOffsetMm_);
}

int ToFArray::readFrontMm()
{
    if (!frontOk_) return -1;

    VL53L0X_RangingMeasurementData_t m;
    sensorFront_.rangingTest(&m, false);

    if (m.RangeStatus != 0) return -1;
    return applyOffsetMm((int)m.RangeMilliMeter, frontOffsetMm_);
}

ToFArray::Readings ToFArray::readAllMm()
{
    Readings result;
    result.leftMm  = readLeftMm();
    result.rightMm = readRightMm();
    result.frontMm = readFrontMm();
    return result;
}

void ToFArray::setOffsetsMm(int16_t leftOffsetMm, int16_t rightOffsetMm, int16_t frontOffsetMm)
{
    leftOffsetMm_  = leftOffsetMm;
    rightOffsetMm_ = rightOffsetMm;
    frontOffsetMm_ = frontOffsetMm;
}

int ToFArray::applyOffsetMm(int distanceMm, int16_t offsetMm)
{
    if (distanceMm < 0) return -1;
    int adjusted = distanceMm - (int)offsetMm;
    return (adjusted < 0) ? 0 : adjusted;
}
