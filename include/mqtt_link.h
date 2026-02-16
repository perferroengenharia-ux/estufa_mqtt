#pragma once
#include <Arduino.h>

struct MqttState {
  const char* id;
  bool systemOn;
  bool heating;
  bool tempValid;
  float tempC;
  float setpoint;
  float u_pct;
  float a1;
  float b0;
  int rssi;
  unsigned long ms;
};

struct MqttCommand {
  char cmd[16];
  bool hasBool;
  bool bVal;
  bool hasNum;
  float fVal;
  char msgId[32];
  char src[16];
};


typedef void (*MqttCmdHandler)(const MqttCommand& c);

void mqtt_set_cmd_handler(MqttCmdHandler h);

void mqtt_begin();
void mqtt_update();

bool mqtt_is_connected();
bool mqtt_just_connected();   // true 1x quando conecta

bool mqtt_publish_state(const MqttState& s);     // retained
bool mqtt_publish_ack(const char* msgId, bool ok, const char* msg = nullptr);
bool mqtt_publish_fault(const char* code, const char* msg);
