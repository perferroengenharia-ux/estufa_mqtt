#pragma once
#include <Arduino.h>
#include "config.h"

static const char* TOPIC_ROOT = "perferro/estufa/v1";

// Monta: perferro/estufa/v1/<ctrl_id>/<suffix>
static inline void topic_make(char* out, size_t n, const char* ctrl_id, const char* suffix) {
  snprintf(out, n, "%s/%s/%s", MQTT_BASE, ctrl_id, suffix);
}

static inline void topic_state(char* out, size_t n, const char* ctrl_id) { topic_make(out, n, ctrl_id, "state"); }
static inline void topic_cmd  (char* out, size_t n, const char* ctrl_id) { topic_make(out, n, ctrl_id, "cmd"); }
static inline void topic_evt  (char* out, size_t n, const char* ctrl_id) { topic_make(out, n, ctrl_id, "evt"); }
static inline void topic_lwt  (char* out, size_t n, const char* ctrl_id) { topic_make(out, n, ctrl_id, "lwt"); }

// wildcard para dashboard (assinatura):
// perferro/estufa/v1/+/state  (dashboard)
// perferro/estufa/v1/+/lwt
// perferro/estufa/v1/+/evt
