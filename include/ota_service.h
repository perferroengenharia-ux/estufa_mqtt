#pragma once
#include <Arduino.h>

// Inicia OTA por URL (HTTP/HTTPS). Roda em task separada.
bool ota_start_url(const char* url, bool reboot_after);

// Status
bool ota_is_running();
