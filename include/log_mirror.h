#pragma once
#include <Arduino.h>

enum LogLvl : uint8_t { LOG_D=0, LOG_I=1, LOG_W=2, LOG_E=3 };

// Inicia fila + (opcional) captura logs do core (ESP_LOGx / WiFiClientSecure etc)
void log_mirror_begin(bool hook_esp_log = true);

// Habilita/desabilita envio por MQTT (Serial continua sempre)
void log_mirror_set_enabled(bool en);
bool log_mirror_is_enabled();

// Define nível mínimo enviado por MQTT
void log_mirror_set_level(LogLvl lvl);
LogLvl log_mirror_get_level();

// Log “app”: imprime no Serial e, se habilitado, envia por MQTT (via fila)
void log_mirror_printf(LogLvl lvl, const char* fmt, ...) __attribute__((format(printf, 2, 3)));

// Deve ser chamado SOMENTE na task de rede (mesma do mqtt.loop), para publicar via MQTT
void log_mirror_poll();

// Converte "D/I/W/E" para enum
LogLvl log_parse_level_char(const char* s);
