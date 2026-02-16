#include "ota_service.h"

#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <ArduinoJson.h>

struct OtaArgs {
  String url;
  bool reboot;
};

static volatile bool g_otaRunning = false;

static void ota_evt(const char* stage, int pct = -1, const char* msg = nullptr) {
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
  ota_evt("START", 0, a->url.c_str());

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  const char* url = a->url.c_str();
  const bool isHttps = (strncmp(url, "https://", 8) == 0);

  bool okBegin = false;
  WiFiClientSecure client;
  if (isHttps) {
    client.setInsecure(); // sem segurança real (barreira/UX)
    okBegin = http.begin(client, url);
  } else {
    okBegin = http.begin(url);
  }

  if (!okBegin) {
    ota_evt("FAIL", -1, "http.begin falhou");
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    char m[64];
    snprintf(m, sizeof(m), "HTTP %d", code);
    ota_evt("FAIL", -1, m);
    http.end();
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  int len = http.getSize(); // pode ser -1 (chunked)
  if (!Update.begin(len > 0 ? (size_t)len : UPDATE_SIZE_UNKNOWN)) {
    ota_evt("FAIL", -1, "Update.begin falhou");
    http.end();
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  WiFiClient* stream = http.getStreamPtr();
  if (!stream) {
    ota_evt("FAIL", -1, "sem stream");
    http.end();
    Update.end();
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  size_t written = 0;
  uint8_t buf[1024];
  int lastPct = -1;

  while (http.connected()) {
    size_t avail = stream->available();
    if (!avail) { delay(1); continue; }

    int toRead = (avail > sizeof(buf)) ? (int)sizeof(buf) : (int)avail;
    int r = stream->readBytes(buf, toRead);
    if (r <= 0) break;

    size_t w = Update.write(buf, (size_t)r);
    if (w != (size_t)r) {
      ota_evt("FAIL", -1, "Update.write erro");
      http.end();
      Update.end();
      g_otaRunning = false;
      delete a;
      vTaskDelete(nullptr);
      return;
    }

    written += w;

    if (len > 0) {
      int pct = (int)((written * 100ULL) / (unsigned long long)len);
      if (pct != lastPct) {
        lastPct = pct;
        ota_evt("DOWNLOADING", pct, nullptr);
      }
    }

    vTaskDelay(1);
  }

  if (!Update.end(true)) {
    ota_evt("FAIL", -1, Update.errorString());
    http.end();
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  http.end();
  ota_evt("DONE", 100, nullptr);

  bool reboot = a->reboot;
  delete a;

  g_otaRunning = false;

  if (reboot) {
    delay(800);
    ESP.restart();
  }

  vTaskDelete(nullptr);
}

bool ota_start_url(const char* url, bool reboot_after) {
  if (!url || !url[0]) return false;
  if (g_otaRunning) return false;

  String u(url);

  // barreira “anti-erro” (não segurança)
  if (!(u.startsWith("http://") || u.startsWith("https://"))) {
    ota_evt("FAIL", -1, "URL deve começar com http/https");
    return false;
  }
  if (!u.endsWith(".bin")) {
    ota_evt("FAIL", -1, "URL nao termina com .bin");
    return false;
  }

  auto* a = new OtaArgs{u, reboot_after};

  BaseType_t ok = xTaskCreate(
    ota_task,
    "ota",
    8192,
    (void*)a,
    1,
    nullptr
  );

  if (ok != pdPASS) {
    delete a;
    return false;
  }

  return true;
}

bool ota_is_running() {
  return g_otaRunning;
}
