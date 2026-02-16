#pragma once

// ====== IDENTIDADE DO CONTROLADOR ======
#ifndef CTRL_ID
  #define CTRL_ID "ctrl01"     // troque em cada placa: ctrl01, ctrl02, ...
#endif

// ====== WIFI ======
#ifndef WIFI_SSID
  #define WIFI_SSID "Perferro"
#endif

#ifndef WIFI_PASS
  #define WIFI_PASS "jv3562507"
#endif

// ====== EMQX CLOUD ======
// Host do EMQX (ex: "abc123.emqxsl.com")
#ifndef MQTT_HOST
  #define MQTT_HOST "kbee663f.ala.us-east-1.emqxsl.com"
#endif

// Porta TLS padrão do EMQX Cloud
#ifndef MQTT_PORT
  #define MQTT_PORT 8883
#endif

#ifndef MQTT_USER
  #define MQTT_USER "perferro"
#endif

#ifndef MQTT_PASS
  #define MQTT_PASS "jv3562507"
#endif

// Padrão mais fácil e que funciona: TLS sem validar certificado
// (depois podemos melhorar com Root CA)
#ifndef MQTT_TLS_INSECURE
  #define MQTT_TLS_INSECURE 1
#endif

// ====== TIMINGS ======
#ifndef WIFI_RECONNECT_MS
  #define WIFI_RECONNECT_MS 3000
#endif

#ifndef MQTT_RECONNECT_MS
  #define MQTT_RECONNECT_MS 3000
#endif

#ifndef MQTT_STATE_PUB_MS
  #define MQTT_STATE_PUB_MS 500   // 0.5s (pode subir p/ 1000ms se quiser)
#endif

#ifndef MQTT_KEEPALIVE_S
  #define MQTT_KEEPALIVE_S 30
#endif

// ====== TOPIC BASE ======
#ifndef MQTT_BASE
  #define MQTT_BASE "perferro/estufa/v1"
#endif
