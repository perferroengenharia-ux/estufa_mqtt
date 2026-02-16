#pragma once
#include <Arduino.h>

// Dependência do seu MQTT: publicar eventos no tópico /evt
bool mqtt_publish_evt(const char* payload, size_t len);

// Inicia OTA por URL (HTTP/HTTPS). Roda em task separada.
bool ota_start_url(const char* url, bool reboot_after);

// Status
bool ota_is_running();
