#include "mqtt_link.h"

#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "config.h"
#include "protocol.h"
#include "wifi_link.h"

static WiFiClientSecure net;
static PubSubClient mqtt(net);

static MqttCmdHandler g_handler = nullptr;

static unsigned long lastTry = 0;
static bool lastConnected = false;
static bool justConnectedFlag = false;

static char t_state[128], t_cmd[128], t_evt[128], t_lwt[128];
static char clientId[64];

static void build_topics() {
  topic_state(t_state, sizeof(t_state), CTRL_ID);
  topic_cmd  (t_cmd,   sizeof(t_cmd),   CTRL_ID);
  topic_evt  (t_evt,   sizeof(t_evt),   CTRL_ID);
  topic_lwt  (t_lwt,   sizeof(t_lwt),   CTRL_ID);
}

static void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // só aceita comandos no tópico cmd
  if (strcmp(topic, t_cmd) != 0) return;

  // copia payload p/ buffer terminando em \0
  static char buf[512];
  if (length >= sizeof(buf)) length = sizeof(buf) - 1;
  memcpy(buf, payload, length);
  buf[length] = '\0';

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, buf);
  if (err) return;

  MqttCommand c;
  memset(&c, 0, sizeof(c));

  const char* cmd = doc["cmd"] | "";
  strncpy(c.cmd, cmd, sizeof(c.cmd) - 1);

  const char* mid = doc["id"] | "";
  strncpy(c.msgId, mid, sizeof(c.msgId) - 1);

  const char* src = doc["src"] | "";
  strncpy(c.src, src, sizeof(c.src) - 1);

  // value pode ser bool ou número
  if (doc.containsKey("value")) {
    if (doc["value"].is<bool>()) {
      c.hasBool = true;
      c.bVal = doc["value"].as<bool>();
    } else if (doc["value"].is<float>() || doc["value"].is<int>()) {
      c.hasNum = true;
      c.fVal = doc["value"].as<float>();
    }
  }

  if (g_handler) g_handler(c);
}

void mqtt_set_cmd_handler(MqttCmdHandler h) {
  g_handler = h;
}

static bool mqtt_connect_now() {
  if (!wifi_is_connected()) return false;

  build_topics();

  // clientId único
  uint64_t mac = ESP.getEfuseMac();
  snprintf(clientId, sizeof(clientId), "%s-%04X%08X",
           CTRL_ID,
           (uint16_t)(mac >> 32),
           (uint32_t)(mac & 0xFFFFFFFF));

#if MQTT_TLS_INSECURE
  net.setInsecure(); // mais fácil (depois podemos usar CA)
  net.setTimeout(2);
  mqtt.setSocketTimeout(2);

#endif

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqtt_callback);
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
  mqtt.setBufferSize(1024);

  // LWT offline retained
  const char* willMsg = "{\"online\":false}";
  bool ok = mqtt.connect(clientId, MQTT_USER, MQTT_PASS, t_lwt, 1, true, willMsg);
  if (!ok) return false;

  // online retained
  mqtt.publish(t_lwt, "{\"online\":true}", true);

  // assina comandos
  mqtt.subscribe(t_cmd, 1);

  return true;
}

void mqtt_begin() {
  lastTry = 0;
  lastConnected = false;
  justConnectedFlag = false;
  build_topics();
}

void mqtt_update() {
  justConnectedFlag = false;

  if (mqtt.connected()) {
    mqtt.loop();
    lastConnected = true;
    return;
  }

  // caiu
  if (lastConnected) lastConnected = false;

  const unsigned long now = millis();
  if (now - lastTry < MQTT_RECONNECT_MS) return;
  lastTry = now;

  if (mqtt_connect_now()) {
    justConnectedFlag = true;
  }
}

bool mqtt_is_connected() {
  return mqtt.connected();
}

bool mqtt_just_connected() {
  return justConnectedFlag;
}

bool mqtt_publish_state(const MqttState& s) {
  if (!mqtt.connected()) return false;

  StaticJsonDocument<512> doc;
  doc["id"] = s.id;
  doc["online"] = true;
  doc["ms"] = s.ms;

  doc["tempC"] = s.tempC;
  doc["tempValid"] = s.tempValid;

  doc["setpoint"] = s.setpoint;
  doc["systemOn"] = s.systemOn;
  doc["heating"] = s.heating;

  doc["u_pct"] = s.u_pct;
  doc["a1"] = s.a1;
  doc["b0"] = s.b0;

  doc["rssi"] = s.rssi;

  char out[512];
  size_t n = serializeJson(doc, out, sizeof(out));

  return mqtt.publish(t_state, (const uint8_t*)out, (unsigned int)n, true);
}

bool mqtt_publish_ack(const char* msgId, bool ok, const char* msg) {
  if (!mqtt.connected()) return false;

  StaticJsonDocument<256> doc;
  doc["type"] = "ack";
  doc["id"] = msgId ? msgId : "";
  doc["ok"] = ok;
  if (msg) doc["msg"] = msg;

  char out[256];
  size_t n = serializeJson(doc, out, sizeof(out));
  return mqtt.publish(t_evt, (const uint8_t*)out, (unsigned int)n, false);
}

bool mqtt_publish_fault(const char* code, const char* msg) {
  if (!mqtt.connected()) return false;

  StaticJsonDocument<256> doc;
  doc["type"] = "fault";
  doc["code"] = code ? code : "";
  doc["msg"]  = msg ? msg : "";

  char out[256];
  size_t n = serializeJson(doc, out, sizeof(out));
  return mqtt.publish(t_evt, (const uint8_t*)out, (unsigned int)n, false);
}