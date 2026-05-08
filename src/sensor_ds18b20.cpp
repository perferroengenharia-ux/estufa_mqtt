#include "sensor_ds18b20.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// --- internos ---
static OneWire* oneWire = nullptr;
static DallasTemperature* sensors = nullptr;

static bool found = false;

static float lastTempC = 0.0f;
static bool tempValid = false;

static bool convPending = false;
static unsigned long tLastReq = 0;
static unsigned long tConvStart = 0;

static uint8_t resBits = 10;
static const unsigned long REDISCOVER_MS = 2000;
static const unsigned long TEMP_PERIOD_MS = 200; // pede conversão periodicamente

static unsigned long conversionTimeMs(uint8_t bits) {
  switch (bits) {
    case 9:  return 94;
    case 10: return 188;
    case 11: return 375;
    default: return 750;
  }
}

static bool temp_is_valid(float t) {
  if (!isfinite(t)) return false;
  if (t == DEVICE_DISCONNECTED_C) return false;
  if (t <= -126.0f) return false;
  if (fabsf(t - 85.0f) < 0.01f) return false;
  if (t < -55.0f || t > 125.0f) return false;
  return true;
}

static void mark_sensor_invalid() {
  tempValid = false;
  found = false;
  convPending = false;
}

// Detecta pelo menos 1 sensor no barramento
static bool find_first_sensor() {
  if (!sensors) return false;
  sensors->begin();
  sensors->setWaitForConversion(false);
  sensors->setResolution(resBits);
  return (sensors->getDeviceCount() >= 1);
}

void sensor_begin(uint8_t pinDQ, uint8_t resolutionBits) {
  resBits = resolutionBits;

  oneWire = new OneWire(pinDQ);
  sensors = new DallasTemperature(oneWire);

  sensors->begin();
  sensors->setWaitForConversion(false); // chave para não travar
  sensors->setResolution(resBits);

  found = find_first_sensor();

  tempValid = false;
  convPending = false;
  tLastReq = millis();
  tConvStart = millis();
}

void sensor_update(unsigned long nowMs) {
  if (!sensors) return;

  // Se não achou, tenta redetectar periodicamente.
  if (!found) {
    static unsigned long lastTry = 0;
    if (nowMs - lastTry >= REDISCOVER_MS) {
      lastTry = nowMs;
      found = find_first_sensor();
      if (found) {
        tempValid = false;
        convPending = false;
        tLastReq = nowMs;
      }
    }
    return;
  }

  // Inicia conversão periodicamente (para o sensor do índice 0)
  if (!convPending && (nowMs - tLastReq) >= TEMP_PERIOD_MS) {
    sensors->requestTemperatures();  // não bloqueia (waitForConversion=false)
    convPending = true;
    tConvStart = nowMs;
    tLastReq = nowMs;
  }

  // Lê após tempo de conversão
  const unsigned long waitMs = conversionTimeMs(resBits);
  if (convPending && (nowMs - tConvStart) >= waitMs) {
    convPending = false;

    float t = sensors->getTempCByIndex(0);
    if (!temp_is_valid(t)) {
      mark_sensor_invalid();
      return;
    }

    lastTempC = t;
    tempValid = true;
  }
}

bool sensor_ok() {
  return found;
}

bool sensor_has_value() {
  return found && tempValid;
}

float sensor_get_c() {
  return lastTempC;
}
