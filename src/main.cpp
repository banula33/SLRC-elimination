
#include <Arduino.h>
#include "ToFArray.h"
#include "RawGyro.h"
#include "MessageHandler.h"

// Define Hardware Pins and Params
// ENA=3, IN1=6, IN2=7, ENB=2, IN3=4, IN4=5
MoveController robot(4, 6, 7, 5, 8, 9, 6.5, 20, 15.0, 255, 50);

// Hardware Serial (UART) bridge
CommProtocol comm(Serial);

// Central handler for routing packets
MessageHandler msgHandler(robot, comm);

// Update these XSHUT pins to match your wiring.
const uint8_t TOF_LEFT_XSHUT_PIN = 26;
const uint8_t TOF_RIGHT_XSHUT_PIN = 22;
const uint8_t TOF_FRONT_XSHUT_PIN = 24;

ToFArray tofArray(TOF_LEFT_XSHUT_PIN, TOF_RIGHT_XSHUT_PIN, TOF_FRONT_XSHUT_PIN);

static ToFArray::Readings lastTof = {-1, -1, -1};
static bool tofReady = false;

static RawGyroReadings lastGyro = {0, 0, 0, 0, 0, 0, 0};
static bool gyroReady = false;

// Faster MPU update, slower ToF update to reduce bus blocking.
const unsigned long GYRO_INTERVAL_MS = 5;
const unsigned long TOF_INTERVAL_MS = 120;
const unsigned long PRINT_INTERVAL_MS = 20;

void setup()
{
  Serial.begin(500000);

  // Initialize motor drivers
  robot.begin();

  // Configure Encoders
  // Right encoder: 23 (A), 25 (B), Left encoder: 27 (A), 29 (B)
  robot.configureEncoders(2, 3, 18, 19);

  // Optionally send a heartbeat or status here to let Brain know we are alive

  setupRawGyro();

  if (!tofArray.begin())
  {
    Serial.println("ToF init failed");
  }
  else
  {
    tofReady = true;
    lastTof = tofArray.readAllMm();
    Serial.println("ToF + RawGyro ready");
  }
}

void loop()
{
  // 1. Maintain hardware states (poll encoders, etc)
  robot.pollEncoders();

  // 2. Poll serial line and dispatch commands
  msgHandler.processIncomingMessages();

  static unsigned long lastGyroMs = 0;
  static unsigned long lastTofMs = 0;
  static unsigned long lastPrintMs = 0;
  unsigned long now = millis();

  if ((now - lastGyroMs) >= GYRO_INTERVAL_MS)
  {
    RawGyroReadings gyro;
    if (updateRawGyro(gyro))
    {
      lastGyro = gyro;
      gyroReady = true;
    }
    lastGyroMs = now;
  }

  if (tofReady && (now - lastTofMs) >= TOF_INTERVAL_MS)
  {
    lastTof = tofArray.readAllMm();
    lastTofMs = now;
  }

  if ((now - lastPrintMs) < PRINT_INTERVAL_MS)
  {
    return;
  }
  lastPrintMs = now;

  // Keep a short delay/yield (or use millis() for better timing logic)
  delay(10);
}
