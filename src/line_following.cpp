#include "line_following.h"

// ─────────────────────────────────────────────────────────────
//  QTR sensor object — shared across all functions.
// ─────────────────────────────────────────────────────────────
static QTRSensors qtr;
static bool initialised_ = false;
static bool calibrated_  = false;

// Scratch buffer for reads
static uint16_t sensorBuf_[LINE_SENSOR_COUNT] = {0};

// ── Init ──────────────────────────────────────────────────────
void initLineSensors()
{
    qtr.setTypeRC();
    qtr.setSensorPins(LINE_SENSOR_PINS, LINE_SENSOR_COUNT);
    initialised_ = true;
    Serial.println(F("Line sensors: pins configured"));
}

// ── Calibration ───────────────────────────────────────────────
// Should be called in setup(). Move the robot by hand over
// BOTH the line and the background during this window.
void calibrateLineSensors(unsigned long durationMs)
{
    if (!initialised_) initLineSensors();

    Serial.println(F("══════════════════════════════════════"));
    Serial.print(F("  LINE SENSOR CALIBRATION — "));
    Serial.print(durationMs / 1000);
    Serial.println(F(" seconds"));
    Serial.println(F("  Move robot over BOTH line and background!"));
    Serial.println(F("══════════════════════════════════════"));

    unsigned long start = millis();
    while ((millis() - start) < durationMs)
    {
        qtr.calibrate();
        delay(10);
    }

    calibrated_ = true;
    printLineSensorCalibration();
    Serial.println(F("Line sensor calibration complete."));
}

bool lineSensorsCalibrated()
{
    return calibrated_;
}

// ── Read line position ────────────────────────────────────────
uint16_t readLinePosition(uint16_t *sensorValues)
{
    if (!calibrated_) return LINE_CENTER_POSITION;

    uint16_t *buf = sensorValues ? sensorValues : sensorBuf_;

    if (LINE_IS_WHITE)
        return qtr.readLineWhite(buf);
    else
        return qtr.readLineBlack(buf);
}

// ── Line detection ────────────────────────────────────────────
bool isOnLine()
{
    if (!calibrated_) return false;

    qtr.readCalibrated(sensorBuf_);

    // When LINE_IS_WHITE: high calibrated value = white (= line)
    // When LINE_IS_BLACK: low calibrated value = black (= line),
    //   but readCalibrated returns 0=min, 1000=max, where max is
    //   the calibration max. For black line on white bg, the line
    //   gives LOW values. So we check < (1000 - threshold).
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (LINE_IS_WHITE)
        {
            // High value = seeing white line
            if (sensorBuf_[i] > LINE_DETECT_THRESHOLD)
                return true;
        }
        else
        {
            // Low value = seeing black line
            if (sensorBuf_[i] < (1000 - LINE_DETECT_THRESHOLD))
                return true;
        }
    }
    return false;
}

// ── Full white bar detection ───────────────────────────────────
// Returns true when ALL sensors are seeing the line colour.
// This means the robot is on a solid horizontal white/black section.
bool isFullWhiteBar()
{
    if (!calibrated_) return false;

    qtr.readCalibrated(sensorBuf_);

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (LINE_IS_WHITE)
        {
            if (sensorBuf_[i] <= LINE_DETECT_THRESHOLD)
                return false; // this sensor is NOT on the white bar
        }
        else
        {
            if (sensorBuf_[i] >= (1000 - LINE_DETECT_THRESHOLD))
                return false; // this sensor is NOT on the black bar
        }
    }
    return true; // all sensors triggered
}

// ── Raw calibrated read ───────────────────────────────────────
bool readLineSensorRaw(uint16_t *out)
{
    if (!calibrated_ || !out) return false;
    qtr.readCalibrated(out);
    return true;
}

// ── Print calibration summary ─────────────────────────────────
void printLineSensorCalibration()
{
    Serial.println(F("Line sensor calibration (min / max):"));
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        Serial.print(F("  S"));
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(F(" / "));
        Serial.println(qtr.calibrationOn.maximum[i]);
    }
    Serial.print(F("  Mode: "));
    Serial.println(LINE_IS_WHITE ? F("WHITE line on dark bg") : F("BLACK line on light bg"));

}
