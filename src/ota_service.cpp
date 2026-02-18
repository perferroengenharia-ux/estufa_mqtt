#include "ota_service.h"

#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <ArduinoJson.h>

#include "mqtt_link.h"

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
  http.setReuse(false);
  http.setTimeout(15000);
  http.useHTTP10(true);

  const char* url = a->url.c_str();
  const bool isHttps = (strncmp(url, "https://", 8) == 0);

  bool ok = false;
  bool updateStarted = false;

  // Mantém o client vivo até o fim
  WiFiClient* client = nullptr;
  WiFiClientSecure* tls = nullptr;

  if (isHttps) {
    tls = new WiFiClientSecure();
    tls->setInsecure();     // sem segurança real
    tls->setTimeout(15000);
    client = tls;
    ok = http.begin(*tls, url);
  } else {
    client = new WiFiClient();
    ok = http.begin(*client, url);
  }

  if (!ok) {
    ota_evt("FAIL", -1, "http.begin falhou");
    http.end();
    if (tls) delete tls;
    else if (client) delete client;
    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  http.addHeader("User-Agent", "PerferroSmartTemp/1.0");

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    char m[96];
    snprintf(m, sizeof(m), "HTTP %d", code);
    ota_evt("FAIL", -1, m);

    http.end();
    if (tls) delete tls;
    else if (client) delete client;

    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  int len = http.getSize(); // pode ser -1 (chunked)
  if (len == 0) {
    ota_evt("FAIL", -1, "size=0");
    http.end();
    if (tls) delete tls;
    else if (client) delete client;

    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  if (!Update.begin(len > 0 ? (size_t)len : UPDATE_SIZE_UNKNOWN)) {
    ota_evt("FAIL", -1, "Update.begin falhou");
    http.end();
    if (tls) delete tls;
    else if (client) delete client;

    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }
  updateStarted = true;

  WiFiClient* stream = http.getStreamPtr();
  if (!stream) {
    ota_evt("FAIL", -1, "sem stream");
    Update.end();
    http.end();
    if (tls) delete tls;
    else if (client) delete client;

    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  stream->setTimeout(15000);

  ota_evt("DOWNLOADING", 0, nullptr);

  size_t written = Update.writeStream(*stream);
  if (written == 0) {
    ota_evt("FAIL", -1, "writeStream=0");
    Update.end();
    http.end();
    if (tls) delete tls;
    else if (client) delete client;

    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  if (len > 0) {
    int pct = (int)((written * 100ULL) / (unsigned long long)len);
    if (pct > 100) pct = 100;
    ota_evt("DOWNLOADING", pct, nullptr);

    if ((int)written != len) {
      char m[96];
      snprintf(m, sizeof(m), "incompleto %u/%d", (unsigned)written, len);
      ota_evt("FAIL", -1, m);
      Update.end();
      http.end();
      if (tls) delete tls;
      else if (client) delete client;

      g_otaRunning = false;
      delete a;
      vTaskDelete(nullptr);
      return;
    }
  }

  if (!Update.end(true)) {
    ota_evt("FAIL", -1, Update.errorString());
    http.end();
    if (tls) delete tls;
    else if (client) delete client;

    g_otaRunning = false;
    delete a;
    vTaskDelete(nullptr);
    return;
  }

  http.end();
  if (tls) delete tls;
  else if (client) delete client;

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
    12288, // <<< mais stack
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
