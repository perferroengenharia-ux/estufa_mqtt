#include "wifi_link.h"
#include <WiFi.h>
#include "config.h"

static unsigned long lastTry = 0;

void wifi_begin() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(CTRL_ID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void wifi_update() {
  if (WiFi.status() == WL_CONNECTED) return;

  const unsigned long now = millis();
  if (now - lastTry < WIFI_RECONNECT_MS) return;
  lastTry = now;

  // reconecta sem travar
  WiFi.reconnect();
}

bool wifi_is_connected() {
  return WiFi.status() == WL_CONNECTED;
}

String wifi_ip() {
  if (!wifi_is_connected()) return String("0.0.0.0");
  return WiFi.localIP().toString();
}

int wifi_rssi() {
  if (!wifi_is_connected()) return -127;
  return WiFi.RSSI();
}
