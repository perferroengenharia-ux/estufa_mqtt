#include <Arduino.h>

#include "sensor_ds18b20.h"
#include "buttons.h"
#include "display_lcd.h"
#include "controlador_caap.h"

#include "config.h"
#include "wifi_link.h"
#include "mqtt_link.h"

#include <Preferences.h>
#include <time.h>
#include <esp_system.h>
#include <ArduinoJson.h>

#include "ota_service.h"


// ======= PINOS =======
static const uint8_t PIN_DS18B20   = 4;
static const uint8_t PIN_SSR       = 26; // BC548 -> SSR

static const uint8_t PIN_BTN_ONOFF = 32;
static const uint8_t PIN_BTN_UP    = 33;
static const uint8_t PIN_BTN_DOWN  = 25;

// LCD I2C
static const uint8_t LCD_ADDR = 0x27;
static const uint8_t LCD_COLS = 16;
static const uint8_t LCD_ROWS = 2;

// ======= Ajustes UI / Processo =======
static const float SP_MIN  = 20.0f;
static const float SP_MAX  = 40.0f;
static const float SP_STEP = 0.5f;

// Controle / LCD / SSR
static const uint32_t CONTROL_UPDATE_MS = 1000;  // controlador 1 Hz
static const uint32_t LCD_UPDATE_MS     = 150;   // LCD
static const uint32_t SSR_TICK_MS       = 10;    // chamada frequente do apply_output
static const uint32_t SERIAL_LOG_MS     = 1000;

// ======= Globais do controle =======
static CAAP_Data meuControle;

// Estado do sistema (precisa ser compartilhado entre tasks)
static volatile bool  g_systemOn = false;
static volatile float g_setpoint = 30.0f;

// Snapshot para publicar no MQTT (telemetria)
static volatile bool  g_tempValid = false;
static volatile float g_tempC     = 0.0f;
static volatile bool  g_heating   = false;

// Mutex leve para proteger o snapshot/variáveis compartilhadas
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ========= HISTÓRICO 24H (1 ponto/hora) =========
struct HistPoint {
  uint32_t ts;   // epoch (segundos). Se não tiver, 0.
  float temp;
};

static Preferences g_prefs;

static HistPoint g_hist[24];
static uint8_t   g_histHead  = 0;   // próxima posição de escrita
static uint8_t   g_histCount = 0;   // 0..24
static uint32_t  g_histLastStoreMs = 0;

// Mutex do histórico (evita race entre tasks)
static SemaphoreHandle_t g_histMutex = nullptr;

// reseta/energia
static bool g_pendingResetEvt = false;
static char g_resetMsg[64] = {0};

// ===== Forward declarations (histórico) =====
static bool time_is_valid();
static uint32_t now_epoch_or_zero();

static void hist_load();
static void hist_save();
static void hist_add_point(float tempC);
static void hist_maybe_store(uint32_t nowMs, bool tempValid, float tempC);
static void hist_publish_all();

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ======= MQTT CMD HANDLER (roda na task de rede via mqtt.loop()) =======
static void on_mqtt_cmd(const MqttCommand& c) {
  // NÃO zere potência/sistema por falta de internet.
  // Só altera quando recebe comando válido.
//===============================================================================
  Serial.printf("[CMD] cmd=%s id=%s src=%s hasStr=%d hasNum=%d hasBool=%d\n",
              c.cmd, c.msgId, c.src, c.hasStr, c.hasNum, c.hasBool);

  if (c.hasStr) {
    Serial.printf("[CMD] url=%s\n", c.sVal);
  }
  if (c.hasReboot) {
    Serial.printf("[CMD] reboot=%d\n", (int)c.reboot);
  }
  //===============================================================================

  if (strcmp(c.cmd, "set_on") == 0 && c.hasBool) {
    portENTER_CRITICAL(&g_mux);
    g_systemOn = c.bVal;
    if (!g_systemOn) meuControle.u_calculado = 0.0f; // desliga na hora se mandou OFF
    portEXIT_CRITICAL(&g_mux);

    mqtt_publish_ack(c.msgId, true);
    return;
  }

  if (strcmp(c.cmd, "set_sp") == 0 && c.hasNum) {
    portENTER_CRITICAL(&g_mux);
    g_setpoint = clampf(c.fVal, SP_MIN, SP_MAX);
    portEXIT_CRITICAL(&g_mux);

    mqtt_publish_ack(c.msgId, true);
    return;
  }

  if (strcmp(c.cmd, "inc_sp") == 0) {
    float step = c.hasNum ? c.fVal : SP_STEP;

    portENTER_CRITICAL(&g_mux);
    g_setpoint = clampf((float)g_setpoint + step, SP_MIN, SP_MAX);
    portEXIT_CRITICAL(&g_mux);

    mqtt_publish_ack(c.msgId, true);
    return;
  }

  if (strcmp(c.cmd, "dec_sp") == 0) {
    float step = c.hasNum ? c.fVal : SP_STEP;

    portENTER_CRITICAL(&g_mux);
    g_setpoint = clampf((float)g_setpoint - step, SP_MIN, SP_MAX);
    portEXIT_CRITICAL(&g_mux);

    mqtt_publish_ack(c.msgId, true);
    return;
  }

  if (strcmp(c.cmd, "req_state") == 0) {
    // Só ACK; a task de rede publica periodicamente de qualquer forma
    mqtt_publish_ack(c.msgId, true);
    return;
  }

  if (strcmp(c.cmd, "req_hist") == 0) {
    // ACK primeiro (opcional) e responde com histórico
    mqtt_publish_ack(c.msgId, true);
    hist_publish_all();
    return;
  }

    if (strcmp(c.cmd, "ota_url") == 0 && c.hasStr) {
    Serial.println("[OTA] comando ota_url recebido, iniciando...");
    mqtt_publish_ack(c.msgId, true);

    bool reboot = true;
    if (c.hasReboot) reboot = c.reboot;

    // inicia OTA em background
    if (!ota_start_url(c.sVal, reboot)) {
      mqtt_publish_ack(c.msgId, false, "falha ao iniciar OTA");
    }
    return;
  }

  mqtt_publish_ack(c.msgId, false, "cmd invalido");
}

// ================= TASK CONTROLE (Core 1) =================
static void taskControle(void* pv) {
  uint32_t lastControl = millis();
  uint32_t lastLcd     = millis();
  uint32_t lastSerial  = millis();

  for (;;) {
    const uint32_t now = millis();

    // 1) Botões (controle local sempre funciona)
    buttons_update(now);

    if (buttons_onoff_event() == EV_PRESS) {
      portENTER_CRITICAL(&g_mux);
      g_systemOn = !g_systemOn;
      if (!g_systemOn) meuControle.u_calculado = 0.0f;
      portEXIT_CRITICAL(&g_mux);
    }

    if (buttons_up_event() != EV_NONE) {
      portENTER_CRITICAL(&g_mux);
      g_setpoint = clampf((float)g_setpoint + SP_STEP, SP_MIN, SP_MAX);
      portEXIT_CRITICAL(&g_mux);
    }

    if (buttons_down_event() != EV_NONE) {
      portENTER_CRITICAL(&g_mux);
      g_setpoint = clampf((float)g_setpoint - SP_STEP, SP_MIN, SP_MAX);
      portEXIT_CRITICAL(&g_mux);
    }

    // 2) Sensor
    sensor_update(now);
    const bool  tempValid = sensor_has_value();
    const float tempC     = sensor_get_c();

    // Histórico 24h (1 ponto/hora)
    hist_maybe_store(now, tempValid, tempC);

    // 3) Controlador 1 Hz
    if (now - lastControl >= CONTROL_UPDATE_MS) {
      lastControl = now;

      bool  localOn;
      float localSp;

      portENTER_CRITICAL(&g_mux);
      localOn = g_systemOn;
      localSp = g_setpoint;
      portEXIT_CRITICAL(&g_mux);

      if (localOn && tempValid) {
        controlador_update(meuControle, tempC, localSp);
      } else {
        // OFF local OU sensor inválido => potência zero
        meuControle.u_calculado = 0.0f;
      }
    }

    // 4) SSR — chamada frequente evita “desligar” se a rede travar
    controlador_apply_output(meuControle, PIN_SSR, 1000);

    const bool heating = (meuControle.u_calculado > 0.5f);

    // Atualiza snapshot para a task de rede publicar
    portENTER_CRITICAL(&g_mux);
    g_tempValid = tempValid;
    g_tempC     = tempC;
    g_heating   = heating;
    portEXIT_CRITICAL(&g_mux);

    // 5) LCD
    if (now - lastLcd >= LCD_UPDATE_MS) {
      lastLcd = now;

      bool  localOn;
      float localSp;
      portENTER_CRITICAL(&g_mux);
      localOn = g_systemOn;
      localSp = g_setpoint;
      portEXIT_CRITICAL(&g_mux);

      display_update(localOn, localSp, tempValid, tempC, heating);
    }

    // 6) Log serial (opcional)
    if (now - lastSerial >= SERIAL_LOG_MS) {
      lastSerial = now;

      bool  localOn;
      float localSp;
      portENTER_CRITICAL(&g_mux);
      localOn = g_systemOn;
      localSp = g_setpoint;
      portEXIT_CRITICAL(&g_mux);

      float u_pct = meuControle.u_calculado;
      if (!localOn || !tempValid) u_pct = 0.0f;

      Serial.print("ID=");
      Serial.print(CTRL_ID);
      Serial.print(" T=");
      Serial.print(tempC, 2);
      Serial.print("C SP=");
      Serial.print(localSp, 2);
      Serial.print(" ON=");
      Serial.print(localOn ? 1 : 0);
      Serial.print(" u=");
      Serial.print(u_pct, 2);
      Serial.print("% a1=");
      Serial.print(meuControle.a1, 6);
      Serial.print(" b0=");
      Serial.println(meuControle.b0, 6);
    }

    vTaskDelay(pdMS_TO_TICKS(SSR_TICK_MS)); // 10ms
  }
}

// ================= TASK REDE (Core 0) =================
static void taskRede(void* pv) {
  uint32_t lastPub = 0;

  // Detecta “borda de conexão” sem depender de mqtt_just_connected()
  bool lastConn = false;

  for (;;) {
    const uint32_t now = millis();

    wifi_update();
    mqtt_update();

    const bool nowConn = mqtt_is_connected();
    if (nowConn && !lastConn) {
      // Conectou agora: tenta NTP e publica RESET pendente
      configTime(0, 0, "pool.ntp.org", "time.nist.gov");

      if (g_pendingResetEvt) {
        mqtt_publish_reset(g_resetMsg);
        g_pendingResetEvt = false;
      }
    }
    lastConn = nowConn;

    // Publica state periodicamente se MQTT estiver conectado
    if (nowConn && (now - lastPub >= MQTT_STATE_PUB_MS)) {
      lastPub = now;

      bool  localOn;
      float localSp;
      bool  tempValid;
      float tempC;
      bool  heating;

      portENTER_CRITICAL(&g_mux);
      localOn   = g_systemOn;
      localSp   = g_setpoint;
      tempValid = g_tempValid;
      tempC     = g_tempC;
      heating   = g_heating;
      portEXIT_CRITICAL(&g_mux);

      float u_pct = meuControle.u_calculado;
      if (!localOn || !tempValid) u_pct = 0.0f;

      MqttState s;
      s.id        = CTRL_ID;
      s.systemOn  = localOn;
      s.heating   = heating;
      s.tempValid = tempValid;
      s.tempC     = tempC;
      s.setpoint  = localSp;
      s.u_pct     = u_pct;
      s.a1        = meuControle.a1;
      s.b0        = meuControle.b0;
      s.rssi      = wifi_rssi(); // ok enviar; app pode ignorar
      s.ms        = now;

      mqtt_publish_state(s);

      if (!tempValid) {
        mqtt_publish_fault("SENSOR", "ds18b20 fail");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Mutex do histórico
  g_histMutex = xSemaphoreCreateMutex();

  // Detecta reset / energia
  esp_reset_reason_t rr = esp_reset_reason();
  const char* rmsg = "RESET";
  switch (rr) {
    case ESP_RST_POWERON:  rmsg = "POWERON"; break;
    case ESP_RST_BROWNOUT: rmsg = "BROWNOUT"; break;
    case ESP_RST_SW:       rmsg = "SW"; break;
    case ESP_RST_PANIC:    rmsg = "PANIC"; break;
    case ESP_RST_WDT:      rmsg = "WDT"; break;
    default:               rmsg = "OTHER"; break;
  }
  snprintf(g_resetMsg, sizeof(g_resetMsg), "%s", rmsg);
  g_pendingResetEvt = true;

  // Garante que o sistema sempre inicia desligado após reboot
  portENTER_CRITICAL(&g_mux);
  g_systemOn = false;
  portEXIT_CRITICAL(&g_mux);

  // Carrega histórico persistido
  hist_load();

  // SSR
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);

  // Módulos do processo (sempre locais)
  buttons_begin(PIN_BTN_ONOFF, PIN_BTN_UP, PIN_BTN_DOWN);

  display_begin(LCD_ADDR, LCD_COLS, LCD_ROWS);
  display_show_boot("SMARTEMP", CTRL_ID);

  sensor_begin(PIN_DS18B20, 10);
  delay(800);
  sensor_update(millis());

  controlador_begin(meuControle, sensor_get_c());

  // Rede
  wifi_begin();
  mqtt_begin();
  mqtt_set_cmd_handler(on_mqtt_cmd);

  // Cria tasks (controle no Core 1, rede no Core 0)
  xTaskCreatePinnedToCore(taskControle, "ctrl", 8192, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskRede,     "net",  8192, nullptr, 1, nullptr, 0);

  display_show_boot("RODANDO LOCAL", "NET EM BACKGND");
}

static bool time_is_valid() {
  time_t now = time(nullptr);
  // "válido" se já passou de 2020-01-01 (aprox)
  return (now > 1577836800);
}

static uint32_t now_epoch_or_zero() {
  if (!time_is_valid()) return 0;
  return (uint32_t)time(nullptr);
}

static void hist_load() {
  if (g_histMutex) xSemaphoreTake(g_histMutex, portMAX_DELAY);

  g_prefs.begin("smarttemp", true);
  g_histHead  = g_prefs.getUChar("h_head", 0);
  g_histCount = g_prefs.getUChar("h_cnt",  0);
  size_t n = g_prefs.getBytesLength("h_blob");

  if (n == sizeof(g_hist)) {
    g_prefs.getBytes("h_blob", g_hist, sizeof(g_hist));
  } else {
    memset(g_hist, 0, sizeof(g_hist));
    g_histHead = 0;
    g_histCount = 0;
  }

  g_prefs.end();

  if (g_histHead > 23) g_histHead = 0;
  if (g_histCount > 24) g_histCount = 24;

  if (g_histMutex) xSemaphoreGive(g_histMutex);
}

static void hist_save() {
  // Observação: hist_save grava flash (NVS).
  // Para manter simples, não trava o mutex aqui porque os valores já foram atualizados
  // com mutex em hist_add_point(). Se preferir, pode travar aqui também.

  g_prefs.begin("smarttemp", false);
  g_prefs.putUChar("h_head", g_histHead);
  g_prefs.putUChar("h_cnt",  g_histCount);
  g_prefs.putBytes("h_blob", g_hist, sizeof(g_hist));
  g_prefs.end();
}

static void hist_add_point(float tempC) {
  if (g_histMutex) xSemaphoreTake(g_histMutex, portMAX_DELAY);

  HistPoint p;
  p.ts   = now_epoch_or_zero();
  p.temp = tempC;

  g_hist[g_histHead] = p;
  g_histHead = (uint8_t)((g_histHead + 1) % 24);
  if (g_histCount < 24) g_histCount++;

  if (g_histMutex) xSemaphoreGive(g_histMutex);

  hist_save();
}

// 1 ponto por hora (e só se temp válida)
static void hist_maybe_store(uint32_t nowMs, bool tempValid, float tempC) {
  if (!tempValid) return;

  if (g_histLastStoreMs == 0) {
    g_histLastStoreMs = nowMs;
    hist_add_point(tempC);
    return;
  }

  if ((nowMs - g_histLastStoreMs) >= 3600000UL) { // 1h
    g_histLastStoreMs = nowMs;
    hist_add_point(tempC);
  }
}

// envia em chunks no formato do app
static void hist_publish_all() {
  // copia snapshot do ring com mutex (evita race)
  HistPoint ordered[24];
  uint8_t n = 0;

  if (g_histMutex) xSemaphoreTake(g_histMutex, portMAX_DELAY);

  n = g_histCount;
  uint8_t start = (g_histCount < 24) ? 0 : g_histHead;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx = (uint8_t)((start + i) % 24);
    ordered[i] = g_hist[idx];
  }

  if (g_histMutex) xSemaphoreGive(g_histMutex);

  const uint8_t CHUNK_SZ = 8;
  uint8_t total = (n + CHUNK_SZ - 1) / CHUNK_SZ;
  if (total == 0) total = 1;

  for (uint8_t seq = 0; seq < total; seq++) {
    StaticJsonDocument<768> doc;
    doc["id"] = CTRL_ID;
    doc["seq"] = seq;
    doc["total"] = total;

    JsonArray points = doc.createNestedArray("points");
    uint8_t from = seq * CHUNK_SZ;
    uint8_t to   = min<uint8_t>(n, from + CHUNK_SZ);

    for (uint8_t i = from; i < to; i++) {
      JsonArray pt = points.createNestedArray();
      pt.add(ordered[i].ts);   // pode ser 0
      pt.add(ordered[i].temp);
    }

    char out[768];
    size_t len = serializeJson(doc, out, sizeof(out));
    mqtt_publish_hist(out, len, false);

    vTaskDelay(pdMS_TO_TICKS(30)); // evita burst muito rápido
  }
}

void loop() {
  // loop vazio: tudo roda nas tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
