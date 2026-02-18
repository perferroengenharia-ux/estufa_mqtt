#include "ota_service.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>

#include "mqtt_link.h"

struct OtaArgs {
  String url;
  bool reboot;
};

static volatile bool g_otaRunning = false;
static bool g_pausedMqttForOta = false;

static void ota_evt(const char* stage, int pct = -1, const char* msg = nullptr) {
  // Serial sempre (importante quando MQTT estiver pausado)
  Serial.print("[OTA] ");
  Serial.print(stage);
  if (pct >= 0) { Serial.print(" "); Serial.print(pct); Serial.print("%"); }
  if (msg && msg[0]) { Serial.print(" - "); Serial.print(msg); }
  Serial.println();

  // Publica EVT só se MQTT estiver conectado
  if (!mqtt_is_connected()) return;

  StaticJsonDocument<256> doc;
  doc["type"]  = "OTA";
  doc["stage"] = stage;
  if (pct >= 0) doc["pct"] = pct;
  if (msg && msg[0]) doc["msg"] = msg;

  char out[256];
  size_t n = serializeJson(doc, out, sizeof(out));
  mqtt_publish_evt(out, n);
}

static void ota_task(void* pv) {
  OtaArgs* a = reinterpret_cast<OtaArgs*>(pv);
  g_otaRunning = true;

  ota_evt("START", 0);

  if (WiFi.status() != WL_CONNECTED) {
    ota_evt("FAIL", -1, "WiFi desconectado");
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  // ===== FIX PRINCIPAL =====
  // Pausa MQTT/TLS durante o OTA para evitar conflito de duas conexões TLS
  // (MQTT 8883 + HTTPS GitHub) que causa SSL internal error (-27648).
  g_pausedMqttForOta = false;
  if (!mqtt_is_paused()) {
    mqtt_pause(true);
    g_pausedMqttForOta = true;
    delay(200);
  }

  Serial.printf("[OTA] free heap=%u\n", (unsigned)ESP.getFreeHeap());
  Serial.print("[OTA] URL: ");
  Serial.println(a->url);

  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(30);
  client.setTimeout(15000);

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(15000);

  if (!http.begin(client, a->url)) {
    ota_evt("FAIL", -1, "http.begin falhou");
    if (g_pausedMqttForOta) mqtt_pause(false);
    g_pausedMqttForOta = false;
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  int httpCode = http.GET();

  // Quando TLS falha, httpCode pode ser <= 0
  if (httpCode <= 0) {
    char msg[64];
    snprintf(msg, sizeof(msg), "GET falhou (%d)", httpCode);
    ota_evt("FAIL", -1, msg);
    http.end();
    if (g_pausedMqttForOta) mqtt_pause(false);
    g_pausedMqttForOta = false;
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  if (httpCode != HTTP_CODE_OK) {
    char msg[32];
    snprintf(msg, sizeof(msg), "HTTP %d", httpCode);
    ota_evt("FAIL", -1, msg);
    http.end();
    if (g_pausedMqttForOta) mqtt_pause(false);
    g_pausedMqttForOta = false;
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  int contentLength = http.getSize();

  if (!Update.begin(contentLength > 0 ? contentLength : UPDATE_SIZE_UNKNOWN)) {
    ota_evt("FAIL", -1, "Update.begin erro");
    http.end();
    if (g_pausedMqttForOta) mqtt_pause(false);
    g_pausedMqttForOta = false;
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  WiFiClient* stream = http.getStreamPtr();

  uint8_t buffer[1024];
  size_t written = 0;
  int lastPct = -1;
  uint32_t lastActivity = millis();

  while (http.connected()) {
    size_t avail = stream->available();

    if (avail) {
      int n = stream->readBytes(buffer, (avail > sizeof(buffer)) ? sizeof(buffer) : avail);
      if (n > 0) {
        Update.write(buffer, n);
        written += (size_t)n;
        lastActivity = millis();

        if (contentLength > 0) {
          int pct = (int)((written * 100UL) / (unsigned long)contentLength);
          if (pct != lastPct) {
            lastPct = pct;
            ota_evt("DOWNLOADING", pct);
          }
          if ((int)written >= contentLength) break;
        }
      }
    } else {
      // timeout de stream (evita loop infinito em conexão ruim)
      if (millis() - lastActivity > 20000) {
        ota_evt("FAIL", -1, "timeout stream");
        Update.abort();
        http.end();
        if (g_pausedMqttForOta) mqtt_pause(false);
        g_pausedMqttForOta = false;
        g_otaRunning = false;
        delete a;
        vTaskDelete(nullptr);
        return;
      }
      vTaskDelay(5);
    }
  }

  if (!Update.end(true)) {
    ota_evt("FAIL", -1, Update.errorString());
    http.end();
    if (g_pausedMqttForOta) mqtt_pause(false);
    g_pausedMqttForOta = false;
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  http.end();
  ota_evt("DONE", 100);

  bool reboot = a->reboot;
  delete a;

  g_otaRunning = false;

  // Se não for reiniciar, retoma MQTT
  if (!reboot && g_pausedMqttForOta) {
    mqtt_pause(false);
  }
  g_pausedMqttForOta = false;

  if (reboot) {
    delay(500);
    ESP.restart();
  }

  vTaskDelete(nullptr);
}

bool ota_start_url(const char* url, bool reboot_after) {
  if (!url || !url[0]) return false;
  if (g_otaRunning) return false;

  String u(url);

  if (!u.startsWith("http")) {
    ota_evt("FAIL", -1, "URL invalida");
    return false;
  }

  if (!u.endsWith(".bin")) {
    ota_evt("FAIL", -1, "Nao termina .bin");
    return false;
  }

  auto* a = new OtaArgs{u, reboot_after};

  if (xTaskCreate(
        ota_task,
        "ota",
        8192,
        a,
        1,
        nullptr
      ) != pdPASS) {
    delete a;
    return false;
  }

  return true;
}

bool ota_is_running() {
  return g_otaRunning;
}
