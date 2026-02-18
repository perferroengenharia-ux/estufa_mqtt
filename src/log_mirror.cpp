#include "log_mirror.h"

#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_log.h>

#include "config.h"
#include "mqtt_link.h"

// ---- Config ----
static const int LOGQ_LEN = 80;
static const int LOG_MSG_MAX = 220;   // manter baixo p/ não estourar buffer MQTT

struct LogItem {
  uint32_t ms;
  uint8_t lvl;
  char msg[LOG_MSG_MAX];
};

static QueueHandle_t g_q = nullptr;

static bool g_enabled = false;     // envio MQTT (Serial sempre)
static uint8_t g_minLvl = LOG_I;

// Hook do ESP-IDF logging (captura logs do core tipo ssl_client.cpp)
static vprintf_like_t g_prev_vprintf = nullptr;

// ---------------- helpers ----------------
static inline const char* lvl_to_char(uint8_t lvl) {
  switch (lvl) {
    case LOG_D: return "D";
    case LOG_I: return "I";
    case LOG_W: return "W";
    case LOG_E: return "E";
    default:    return "I";
  }
}

LogLvl log_parse_level_char(const char* s) {
  if (!s || !s[0]) return LOG_I;
  char c = s[0];
  if (c=='D' || c=='d') return LOG_D;
  if (c=='I' || c=='i') return LOG_I;
  if (c=='W' || c=='w') return LOG_W;
  if (c=='E' || c=='e') return LOG_E;
  return LOG_I;
}

static void enqueue_line(uint8_t lvl, const char* line) {
  if (!g_q) return;
  if (!g_enabled) return;
  if (lvl < g_minLvl) return;

  LogItem it;
  it.ms = millis();
  it.lvl = lvl;
  it.msg[0] = '\0';

  if (line && line[0]) {
    strlcpy(it.msg, line, sizeof(it.msg));
  } else {
    strlcpy(it.msg, "(empty)", sizeof(it.msg));
  }

  // não bloquear: se lotar, dropa
  xQueueSend(g_q, &it, 0);
}

// Hook captura o “texto final” que iria pro Serial em logs do core
static int my_vprintf(const char* fmt, va_list args) {
  // 1) chama o output original (pra continuar aparecendo no Serial)
  int r = 0;
  if (g_prev_vprintf) {
    va_list c1;
    va_copy(c1, args);
    r = g_prev_vprintf(fmt, c1);
    va_end(c1);
  }

  // 2) captura cópia formatada e joga na fila (se habilitado)
  if (g_enabled && g_q) {
    char buf[LOG_MSG_MAX];
    va_list c2;
    va_copy(c2, args);
    vsnprintf(buf, sizeof(buf), fmt, c2);
    va_end(c2);

    // Heurística de nível: muitos logs já vêm com "[E]" etc.
    uint8_t lvl = LOG_I;
    if (strstr(buf, "[E]") || strstr(buf, " E ") || strstr(buf, "error") ) lvl = LOG_E;
    else if (strstr(buf, "[W]") || strstr(buf, " W ") || strstr(buf, "warn")) lvl = LOG_W;
    else if (strstr(buf, "[D]") || strstr(buf, " D ") ) lvl = LOG_D;

    enqueue_line(lvl, buf);
  }

  return r;
}

void log_mirror_begin(bool hook_esp_log) {
  if (!g_q) {
    g_q = xQueueCreate(LOGQ_LEN, sizeof(LogItem));
  }

  if (hook_esp_log) {
    // instala hook para capturar logs do core (ex.: ssl_client.cpp)
    g_prev_vprintf = esp_log_set_vprintf(&my_vprintf);
  }
}

void log_mirror_set_enabled(bool en) { g_enabled = en; }
bool log_mirror_is_enabled() { return g_enabled; }

void log_mirror_set_level(LogLvl lvl) { g_minLvl = (uint8_t)lvl; }
LogLvl log_mirror_get_level() { return (LogLvl)g_minLvl; }

void log_mirror_printf(LogLvl lvl, const char* fmt, ...) {
  // sempre imprime no Serial (comportamento atual)
  char buf[LOG_MSG_MAX];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Serial.println(buf);

  // fila para MQTT (publicação só na task de rede)
  enqueue_line((uint8_t)lvl, buf);
}

void log_mirror_poll() {
  if (!g_q) return;

  // Só publica via MQTT se conectado e não pausado (OTA pausa MQTT)
  if (!mqtt_is_connected()) return;
  if (mqtt_is_paused()) return;
  if (!g_enabled) {
    // se não habilitado, drena para não acumular
    LogItem dump;
    while (xQueueReceive(g_q, &dump, 0) == pdTRUE) {}
    return;
  }

  // publica até N por ciclo para não travar loop
  for (int i = 0; i < 10; i++) {
    LogItem it;
    if (xQueueReceive(g_q, &it, 0) != pdTRUE) break;

    StaticJsonDocument<384> doc;
    doc["type"] = "LOG";
    doc["id"]   = CTRL_ID;
    doc["ms"]   = it.ms;
    doc["lvl"]  = lvl_to_char(it.lvl);
    doc["msg"]  = it.msg;

    char out[384];
    size_t n = serializeJson(doc, out, sizeof(out));
    mqtt_publish_evt(out, n);
  }
}
