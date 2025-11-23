#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "pico/multicore.h"

// ---------------------- USER CONFIG ----------------------

// WS2812B LED strip settings
#define LED_PIN     2          // GPIO pin for data line to WS2812B
#define LED_COUNT   24         // Number of LEDs in the strip

// I2C / MPU6050
#define I2C_SDA_PIN  4         // Change to your hat wiring if needed
#define I2C_SCL_PIN  5
#define MPU_ADDR     0x68

// Update rates
const uint16_t SENSOR_INTERVAL_MS = 10;   // Core0: sensor / simulation update period
const uint16_t FRAME_INTERVAL_MS  = 50;   // Core1: LED + ASCII frame rate (~20 FPS)

// ---------------------- GLOBAL STATE ----------------------

struct MotionSample {
  float ax, ay, az;    // acceleration in g
  float gx, gy, gz;    // gyro in deg/s (not heavily used here)
  float rollDeg;       // left/right tilt angle in degrees (for water)
  bool  sensorOk;      // true if data from real sensor, false if simulated
};

// FIX: not volatile, so we can copy-assign without compiler complaining
MotionSample g_motion = {0, 0, 1, 0, 0, 0, 0, false};
volatile bool g_mpuPresent = false;

// LED strip object (Core1 will use it)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------- MPU6050 LOW-LEVEL ----------------------

bool mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool mpuReadBytes(uint8_t startReg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t readCount = Wire.requestFrom(MPU_ADDR, length);
  if (readCount != length) {
    return false;
  }
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool initMPU6050() {
  // Simple presence check
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Wake up the MPU6050 (clear sleep bit in PWR_MGMT_1)
  if (!mpuWriteRegister(0x6B, 0x00)) {
    return false;
  }

  // Optional: set accelerometer ±2g, gyro ±250 dps (default)
  // ACCEL_CONFIG (0x1C), GYRO_CONFIG (0x1B) if you want to be explicit.

  return true;
}

// Reads accel/gyro, computes roll angle (tilt left/right)
bool readMPU6050(MotionSample &out) {
  uint8_t data[14];
  if (!mpuReadBytes(0x3B, data, 14)) {
    return false;
  }

  int16_t rawAx = (data[0] << 8) | data[1];
  int16_t rawAy = (data[2] << 8) | data[3];
  int16_t rawAz = (data[4] << 8) | data[5];
  // int16_t rawTemp = (data[6] << 8) | data[7]; // not used
  int16_t rawGx = (data[8] << 8) | data[9];
  int16_t rawGy = (data[10] << 8) | data[11];
  int16_t rawGz = (data[12] << 8) | data[13];

  // Convert to physical units
  const float ACCEL_SCALE = 16384.0f; // LSB/g for ±2g
  const float GYRO_SCALE  = 131.0f;   // LSB/(deg/s) for ±250 dps

  out.ax = (float)rawAx / ACCEL_SCALE;
  out.ay = (float)rawAy / ACCEL_SCALE;
  out.az = (float)rawAz / ACCEL_SCALE;

  out.gx = (float)rawGx / GYRO_SCALE;
  out.gy = (float)rawGy / GYRO_SCALE;
  out.gz = (float)rawGz / GYRO_SCALE;

  // Compute roll angle (rotation around X axis, left/right tilt)
  // roll = atan2(Ay, Az)
  float rollRad = atan2(out.ay, out.az);
  out.rollDeg = rollRad * 180.0f / PI;

  out.sensorOk = true;
  return true;
}

// ---------------------- SIMULATED MOTION (NO SENSOR) ----------------------

void generateSimulatedMotion(MotionSample &out) {
  // Simple smooth "rocking" motion using sine wave
  float t = millis() / 1000.0f;
  out.ax = 0.0f;
  out.ay = 0.0f;
  out.az = 1.0f;

  out.gx = 0.0f;
  out.gy = 0.0f;
  out.gz = 0.0f;

  out.rollDeg = 30.0f * sinf(t * 0.8f); // ±30°
  out.sensorOk = false;
}

// ---------------------- WATER RENDERING (CORE 1) ----------------------

// Map roll angle (deg) to normalized tilt [-1, 1]
float mapRollToTilt(float rollDeg) {
  // Clamp roll to ±45°
  float clamped = constrain(rollDeg, -45.0f, 45.0f);
  return clamped / 45.0f; // -1 at -45°, +1 at +45°
}

// Update physical LEDs to show water "blob"
void updateLeds(float tiltNorm) {
  if (LED_COUNT <= 0) return;

  int mid = LED_COUNT / 2;
  // Shift center with tilt
  float maxShift = (LED_COUNT / 2) - 1;
  float centerF = mid + tiltNorm * maxShift;

  // Simple edge "bounce" illusion: if center goes beyond edges, reflect
  if (centerF < 0) {
    centerF = -centerF;  // reflect at left
  } else if (centerF > LED_COUNT - 1) {
    centerF = 2 * (LED_COUNT - 1) - centerF; // reflect at right
  }

  // Soft radius of water
  const float RADIUS = LED_COUNT * 0.25f; // 1/4 of strip

  for (int i = 0; i < LED_COUNT; i++) {
    float dist = fabsf(i - centerF);
    float base = 0.0f;

    if (dist < RADIUS) {
      // Simple triangular falloff
      base = 1.0f - (dist / RADIUS);
    } else {
      base = 0.0f;
    }

    // Add a tiny shimmer by modulating with time
    float t = millis() / 1000.0f;
    float shimmer = 0.15f * (0.5f + 0.5f * sinf(t * 8.0f + i * 0.6f));

    float brightness = constrain(base + shimmer, 0.0f, 1.0f);

    // Water: blue-ish, with a hint of green
    uint8_t r = 0;
    uint8_t g = (uint8_t)(brightness * 50.0f);
    uint8_t b = (uint8_t)(brightness * 180.0f);

    strip.setPixelColor(i, strip.Color(r, g, b));
  }

  strip.show();
}

// Build ASCII representation of water for Serial Monitor
String buildAsciiWaterLine(float tiltNorm) {
  int length = LED_COUNT > 0 ? LED_COUNT : 20; // fallback to 20 chars if LED_COUNT invalid
  String s;
  s.reserve(length);

  int mid = length / 2;
  float maxShift = (length / 2) - 1;
  float centerF = mid + tiltNorm * maxShift;

  // Same reflection trick as LED
  if (centerF < 0) {
    centerF = -centerF;
  } else if (centerF > length - 1) {
    centerF = 2 * (length - 1) - centerF;
  }

  const float RADIUS = length * 0.25f;

  for (int i = 0; i < length; i++) {
    float dist = fabsf(i - centerF);
    if (dist < 0.5f) {
      // Center of the water blob
      s += 'O';         // strong center
    } else if (dist < RADIUS * 0.4f) {
      s += '~';         // wavy water core
    } else if (dist < RADIUS) {
      s += '-';         // weaker rim
    } else {
      s += '_';         // empty
    }
  }

  return s;
}

// ---------------------- CORE 1 TASK ----------------------

void core1Task() {
  // Small delay to let Core0 finish setup
  delay(1500);

  while (true) {
    // Copy shared motion (single struct copy; fine for this use case)
    MotionSample local;
    noInterrupts();
    local = g_motion;
    interrupts();

    float tiltNorm = mapRollToTilt(local.rollDeg);

    // Update LEDs
    updateLeds(tiltNorm);

    // Build ASCII water line
    String water = buildAsciiWaterLine(tiltNorm);

    // Print one line per frame: sensor values + ASCII
    Serial.print("roll=");
    Serial.print(local.rollDeg, 1);
    Serial.print(" deg");

    Serial.print(" | ax=");
    Serial.print(local.ax, 2);
    Serial.print("g ay=");
    Serial.print(local.ay, 2);
    Serial.print("g az=");
    Serial.print(local.az, 2);
    Serial.print("g");

    Serial.print(" | src=");
    Serial.print(local.sensorOk ? "MPU" : "SIM");

    Serial.print(" | water: ");
    Serial.println(water);   // ONE line per frame, no flicker

    delay(FRAME_INTERVAL_MS);
  }
}

// ---------------------- ARDUINO CORE 0 (setup/loop) ----------------------

void setup() {
  Serial.begin(115200);
  delay(1000);

  // I2C setup
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // Try to init MPU6050
  g_mpuPresent = initMPU6050();

  if (g_mpuPresent) {
    Serial.println("MPU-6050 detected, using real sensor.");
  } else {
    Serial.println("MPU-6050 NOT found, falling back to simulated motion.");
  }

  // LED strip init
  strip.begin();
  strip.clear();
  strip.show();

  // Start second core
  multicore_launch_core1(core1Task);
}

void loop() {
  static uint32_t lastSampleMs = 0;
  uint32_t now = millis();

  if (now - lastSampleMs >= SENSOR_INTERVAL_MS) {
    lastSampleMs = now;

    MotionSample sample;

    if (g_mpuPresent && readMPU6050(sample)) {
      // OK, real sensor data
      g_motion = sample;
    } else {
      // Either no sensor at all or read failed -> simulate
      generateSimulatedMotion(sample);
      g_motion = sample;
    }
  }

  // Core0 doesn't need to do anything else
}
