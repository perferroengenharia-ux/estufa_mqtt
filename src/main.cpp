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
#include <math.h>
#include <strings.h>

#include "ota_service.h"

#include "log_mirror.h"

#include <esp_task_wdt.h>

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
static const uint32_t CONTROL_FAILSAFE_MS = 3000;

// ALERTAS LCD
static volatile bool g_alertReset = false;     // queda energia / reset
static volatile bool g_alertSensor = false;    // sensor falhando
static uint32_t g_sensorFailSinceMs = 0;

static const uint32_t SERIAL_LOG_MS     = 1000;

// ======= Globais do controle =======
static CAAP_Data meuControle;

// Estado do sistema (precisa ser compartilhado entre tasks)
static volatile bool  g_systemOn = false;
static volatile float g_setpoint = 32.0f;

// Snapshot para publicar no MQTT (telemetria)
static volatile bool  g_tempValid = false;
static volatile float g_tempC     = 0.0f;
static volatile bool  g_heating   = false;
static volatile uint32_t g_lastControlKickMs = 0;

// Mutex leve para proteger o snapshot/variáveis compartilhadas
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t g_controlMutex = nullptr;

// ========= HISTÓRICO 24H (1 ponto/hora) =========
struct HistPoint {
  uint32_t ts;   // epoch (segundos). Se não tiver, 0.
  float temp;
};

static const uint32_t HIST_TS_REL_FLAG = 0x80000000UL;

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
static bool temp_is_valid_for_history(float t);

static void hist_load();
static void hist_save();
static void hist_add_point(float tempC);
static void hist_maybe_store(uint32_t nowMs, bool tempValid, float tempC);
static void hist_publish_all(const char* reqId);

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static bool control_lock(TickType_t ticks = pdMS_TO_TICKS(50)) {
  if (!g_controlMutex) return true;
  return xSemaphoreTake(g_controlMutex, ticks) == pdTRUE;
}

static void control_unlock() {
  if (g_controlMutex) xSemaphoreGive(g_controlMutex);
}

static void control_force_output_off() {
  if (control_lock()) {
    meuControle.u_calculado = 0.0f;
    control_unlock();
  }
  digitalWrite(PIN_SSR, LOW);
}

static CAAP_Data control_snapshot() {
  CAAP_Data snap;
  memset(&snap, 0, sizeof(snap));

  if (control_lock()) {
    snap = meuControle;
    control_unlock();
  }

  if (!isfinite(snap.u_calculado) || snap.u_calculado < 0.0f) snap.u_calculado = 0.0f;
  if (snap.u_calculado > 100.0f) snap.u_calculado = 100.0f;
  return snap;
}

static void control_update_locked(float tempC, float setpoint) {
  if (!control_lock(portMAX_DELAY)) return;
  controlador_update(meuControle, tempC, setpoint);
  control_unlock();
}

static bool parse_bool_text(const char* text, bool* out) {
  if (!text || !out) return false;

  if (strcasecmp(text, "true") == 0 || strcasecmp(text, "on") == 0 ||
      strcasecmp(text, "1") == 0 || strcasecmp(text, "yes") == 0) {
    *out = true;
    return true;
  }

  if (strcasecmp(text, "false") == 0 || strcasecmp(text, "off") == 0 ||
      strcasecmp(text, "0") == 0 || strcasecmp(text, "no") == 0) {
    *out = false;
    return true;
  }

  return false;
}

static bool command_bool_value(const MqttCommand& c, bool* out) {
  if (!out) return false;

  if (c.hasBool) {
    *out = c.bVal;
    return true;
  }

  if (c.hasNum) {
    *out = (c.fVal != 0.0f);
    return true;
  }

  if (c.hasStr) {
    return parse_bool_text(c.sVal, out);
  }

  return false;
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

  if (strcmp(c.cmd, "set_on") == 0) {
    bool requestedOn = false;
    if (!command_bool_value(c, &requestedOn)) {
      mqtt_publish_ack(c.msgId, false, "value ausente/invalido");
      log_mirror_printf(LOG_W, "[CMD] set_on ignorado: value invalido id=%s src=%s", c.msgId, c.src);
      return;
    }

    const bool turnOff = !requestedOn;

    portENTER_CRITICAL(&g_mux);
    g_systemOn = requestedOn;
    if (requestedOn) g_alertReset = false;
    portEXIT_CRITICAL(&g_mux);

    if (turnOff) control_force_output_off();

    mqtt_publish_ack(c.msgId, true);
    log_mirror_printf(LOG_I, "[CMD] set_on=%d aplicado id=%s src=%s", requestedOn ? 1 : 0, c.msgId, c.src);
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
    hist_publish_all(c.msgId);
    return;
  }

  // ======= OTA URL (ALTERAÇÃO MÍNIMA AQUI) =======
  if (strcmp(c.cmd, "ota_url") == 0 && c.hasStr) {
    log_mirror_printf(LOG_I, "[OTA] comando ota_url recebido, iniciando...");

    bool reboot = true;
    if (c.hasReboot) reboot = c.reboot;

    // Inicia OTA em background.
    // Só responde ACK OK se realmente conseguiu disparar a task do OTA.
    if (ota_start_url(c.sVal, reboot)) {
      mqtt_publish_ack(c.msgId, true);
    } else {
      mqtt_publish_ack(c.msgId, false, "falha ao iniciar OTA");
    }
    return;
  }
  // ==============================================

  if (strcmp(c.cmd, "log_set") == 0 && c.hasBool) {
  log_mirror_set_enabled(c.bVal);
  mqtt_publish_ack(c.msgId, true);
  return;
}

if (strcmp(c.cmd, "log_level") == 0 && (c.hasStr || c.hasNum)) {
  char lvlText[8];
  if (c.hasStr) {
    strlcpy(lvlText, c.sVal, sizeof(lvlText));
  } else {
    snprintf(lvlText, sizeof(lvlText), "%d", (int)c.fVal);
  }
  LogLvl lvl = log_parse_level_char(lvlText); // aceita D/I/W/E, debug/info/warn/error ou 0..3
  log_mirror_set_level(lvl);
  mqtt_publish_ack(c.msgId, true);
  return;
}

  mqtt_publish_ack(c.msgId, false, "cmd invalido");
}

// ================= TASK CONTROLE (Core 1) =================
static void taskControle(void* pv) {
  esp_task_wdt_add(NULL);
  esp_task_wdt_reset();

  uint32_t lastControl = millis();
  uint32_t lastLcd     = millis();
  uint32_t lastSerial  = millis();
  uint32_t lastSensorWarnLog = 0;
  uint32_t lastSensorErrorLog = 0;
  bool sensorFaultWasActive = false;

  for (;;) {
    esp_task_wdt_reset();
    const uint32_t now = millis();

    // 1) Botões (controle local sempre funciona)
    buttons_update(now);

    if (buttons_onoff_event() == EV_PRESS) {
      bool nowOn;
      portENTER_CRITICAL(&g_mux);
      g_systemOn = !g_systemOn;
      nowOn = g_systemOn;
      portEXIT_CRITICAL(&g_mux);

      if (!nowOn) control_force_output_off();
    }

    // Se ligou, reconhece o alerta de reset
    if (g_systemOn) g_alertReset = false;

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

    if (!tempValid) {
      control_force_output_off();
    }

    // Falha de sensor: só considera erro se ficar inválido por > 3s
    if (!tempValid) {
      if (g_sensorFailSinceMs == 0) g_sensorFailSinceMs = now;
      if ((now - g_sensorFailSinceMs) > 3000) {
        g_alertSensor = true;
        sensorFaultWasActive = true;

        bool localOnForSensorLog;
        portENTER_CRITICAL(&g_mux);
        localOnForSensorLog = g_systemOn;
        portEXIT_CRITICAL(&g_mux);

        if (localOnForSensorLog) {
          if (lastSensorErrorLog == 0 || (now - lastSensorErrorLog) >= 10000UL) {
            lastSensorErrorLog = now;
            log_mirror_printf(LOG_E, "ERRO SENSOR: DS18B20 invalido; aquecimento bloqueado");
          }
        } else if (lastSensorWarnLog == 0 || (now - lastSensorWarnLog) >= 30000UL) {
          lastSensorWarnLog = now;
          log_mirror_printf(LOG_W, "WARN SENSOR: DS18B20 ausente/invalido; SSR desligado");
        }
      }
    } else {
      if (sensorFaultWasActive) {
        log_mirror_printf(LOG_I, "SENSOR OK: DS18B20 recuperado T=%.2fC", tempC);
      }
      sensorFaultWasActive = false;
      lastSensorWarnLog = 0;
      lastSensorErrorLog = 0;
      g_sensorFailSinceMs = 0;
      g_alertSensor = false;
    }

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
        control_update_locked(tempC, localSp);
      } else {
        // OFF local OU sensor inválido => potência zero
        control_force_output_off();
      }

      g_lastControlKickMs = now;
    }

    // 4) SSR — chamada frequente evita “desligar” se a rede travar
    bool localOnForOutput;
    portENTER_CRITICAL(&g_mux);
    localOnForOutput = g_systemOn;
    portEXIT_CRITICAL(&g_mux);

    const uint32_t lastKick = g_lastControlKickMs;
    const bool controlStale = (lastKick == 0) || ((now - lastKick) > CONTROL_FAILSAFE_MS);
    CAAP_Data ctrlSnap = control_snapshot();
    bool heating = false;

    if (!localOnForOutput || !tempValid || controlStale) {
      control_force_output_off();
      ctrlSnap.u_calculado = 0.0f;
    } else {
      controlador_apply_output(ctrlSnap, PIN_SSR, 1000);
      heating = (ctrlSnap.u_calculado > 0.5f);
    }

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

      // PRIORIDADE: RESET > SENSOR > NORMAL
      if (g_alertReset) {
        display_set_alert(true, "!! RESET/ENERGIA", "LIGUE NOVAMENTE!", true);
      } else if (g_alertSensor) {
        display_set_alert(true, "ERRO SENSOR", "DS18B20 FALHA", false);
      } else {
        display_set_alert(false, "", "", false);
      }

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

      CAAP_Data logSnap = control_snapshot();
      float u_pct = logSnap.u_calculado;
      if (!localOn || !tempValid) u_pct = 0.0f;

      char tempText[16];
      if (tempValid) {
        snprintf(tempText, sizeof(tempText), "%.2f", tempC);
      } else {
        strlcpy(tempText, "--.--", sizeof(tempText));
      }

      log_mirror_printf(LOG_D,
      "ID=%s T=%sC TV=%d SP=%.2f ON=%d u=%.2f%% a1=%.6f b0=%.6f",
      CTRL_ID, tempText, tempValid ? 1 : 0, localSp, localOn ? 1 : 0, u_pct, logSnap.a1, logSnap.b0);

    }

    vTaskDelay(pdMS_TO_TICKS(SSR_TICK_MS)); // 10ms
  }
}

// ================= TASK REDE (Core 0) =================
static void taskRede(void* pv) {
  esp_task_wdt_add(NULL);
  esp_task_wdt_reset();

  uint32_t lastPub = 0;
  uint32_t lastSensorFaultPub = 0;
  bool lastSensorFaultState = false;

  // Detecta “borda de conexão” sem depender de mqtt_just_connected()
  bool lastConn = false;

  for (;;) {
    esp_task_wdt_reset();
    const uint32_t now = millis();

    wifi_update();
    mqtt_update();

    log_mirror_poll(); // publica logs enfileirados via MQTT (somente aqui!)

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

      CAAP_Data netSnap = control_snapshot();
      float u_pct = netSnap.u_calculado;
      if (!localOn || !tempValid) u_pct = 0.0f;

      MqttState s;
      s.id        = CTRL_ID;
      s.systemOn  = localOn;
      s.heating   = heating;
      s.tempValid = tempValid;
      s.tempC     = tempC;
      s.setpoint  = localSp;
      s.u_pct     = u_pct;
      s.a1        = netSnap.a1;
      s.b0        = netSnap.b0;
      s.rssi      = wifi_rssi(); // ok enviar; app pode ignorar
      s.ms        = now;

      mqtt_publish_state(s);

      if (!tempValid && (!lastSensorFaultState || (now - lastSensorFaultPub >= 10000UL))) {
        mqtt_publish_fault("SENSOR", "ds18b20 fail");
        lastSensorFaultPub = now;
      }
      lastSensorFaultState = !tempValid;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  esp_task_wdt_init(10, true);   // timeout 10 s, reinicia se expirar
  disableLoopWDT();
  if (esp_task_wdt_status(NULL) == ESP_OK) {
    esp_task_wdt_delete(NULL);
  }

  log_mirror_begin(true); // true = captura logs do core (ssl_client.cpp etc)

  // Mutex do histórico
  g_histMutex = xSemaphoreCreateMutex();
  g_controlMutex = xSemaphoreCreateMutex();

  // SSR em estado seguro o mais cedo possivel no boot.
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);

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
  // Mostra urgente no LCD até o usuário ligar novamente
  g_alertReset = true;

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
  display_show_boot("PERFERRO CONTROL", CTRL_ID);

  sensor_begin(PIN_DS18B20, 10);
  uint32_t sensorPrimeStart = millis();
  while (millis() - sensorPrimeStart < 300UL) {
    sensor_update(millis());
    delay(10);
  }

  const float initialTemp = sensor_has_value() ? sensor_get_c() : 25.0f;
  controlador_begin(meuControle, initialTemp);
  control_force_output_off();
  g_lastControlKickMs = millis();

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

static uint32_t hist_now_timestamp() {
  if (time_is_valid()) return (uint32_t)time(nullptr);

  uint32_t rel = (uint32_t)(millis() / 1000UL);
  if (rel == 0) rel = 1;
  return (rel & ~HIST_TS_REL_FLAG) | HIST_TS_REL_FLAG;
}

static bool hist_ts_is_relative(uint32_t ts) {
  return (ts & HIST_TS_REL_FLAG) != 0;
}

static uint32_t hist_ts_publish_value(uint32_t ts) {
  return hist_ts_is_relative(ts) ? (ts & ~HIST_TS_REL_FLAG) : ts;
}

static bool temp_is_valid_for_history(float t) {
  if (!isfinite(t)) return false;
  if (t <= -126.0f) return false;
  if (fabsf(t - 85.0f) < 0.01f) return false;
  if (t < -55.0f || t > 125.0f) return false;
  return true;
}

static void hist_load() {
  if (g_histMutex) xSemaphoreTake(g_histMutex, portMAX_DELAY);

  Preferences prefs;
  if (prefs.begin("smarttemp", false)) {
    g_histHead  = prefs.getUChar("h_head", 0);
    g_histCount = prefs.getUChar("h_cnt",  0);
    size_t n = prefs.isKey("h_blob") ? prefs.getBytesLength("h_blob") : 0;

    if (n == sizeof(g_hist)) {
      prefs.getBytes("h_blob", g_hist, sizeof(g_hist));
    } else {
      memset(g_hist, 0, sizeof(g_hist));
      g_histHead = 0;
      g_histCount = 0;
    }

    prefs.end();
  } else {
    memset(g_hist, 0, sizeof(g_hist));
    g_histHead = 0;
    g_histCount = 0;
  }

  if (g_histHead > 23) g_histHead = 0;
  if (g_histCount > 24) g_histCount = 24;

  if (g_histMutex) xSemaphoreGive(g_histMutex);
}

static void hist_save() {
  // Snapshot protegido; a gravacao em NVS usa a copia local.
  HistPoint histCopy[24];
  uint8_t headCopy = 0;
  uint8_t countCopy = 0;

  if (g_histMutex) xSemaphoreTake(g_histMutex, portMAX_DELAY);
  memcpy(histCopy, g_hist, sizeof(histCopy));
  headCopy = g_histHead;
  countCopy = g_histCount;
  if (g_histMutex) xSemaphoreGive(g_histMutex);

  Preferences prefs;
  if (!prefs.begin("smarttemp", false)) return;

  prefs.putUChar("h_head", headCopy);
  prefs.putUChar("h_cnt",  countCopy);
  prefs.putBytes("h_blob", histCopy, sizeof(histCopy));
  prefs.end();
}

static void hist_add_point(float tempC) {
  if (!temp_is_valid_for_history(tempC)) return;

  if (g_histMutex) xSemaphoreTake(g_histMutex, portMAX_DELAY);

  HistPoint p;
  p.ts   = hist_now_timestamp();
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
  if (!temp_is_valid_for_history(tempC)) return;

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
static void hist_publish_all(const char* reqId) {
  // copia snapshot do ring com mutex (evita race)
  HistPoint ordered[24];
  uint8_t n = 0;

  if (g_histMutex) xSemaphoreTake(g_histMutex, portMAX_DELAY);

  uint8_t stored = g_histCount;
  uint8_t start = (g_histCount < 24) ? 0 : g_histHead;
  for (uint8_t i = 0; i < stored; i++) {
    uint8_t idx = (uint8_t)((start + i) % 24);
    if (g_hist[idx].ts == 0) continue;
    if (!temp_is_valid_for_history(g_hist[idx].temp)) continue;
    ordered[n++] = g_hist[idx];
  }

  if (g_histMutex) xSemaphoreGive(g_histMutex);

  const uint8_t CHUNK_SZ = 8;
  uint8_t total = (n + CHUNK_SZ - 1) / CHUNK_SZ;
  if (total == 0) total = 1;

  for (uint8_t seq = 0; seq < total; seq++) {
    StaticJsonDocument<768> doc;
    doc["id"] = CTRL_ID;
    doc["req_id"] = reqId ? reqId : "";
    doc["seq"] = seq;
    doc["total"] = total;

    JsonArray points = doc.createNestedArray("points");
    uint8_t from = seq * CHUNK_SZ;
    uint8_t to   = min<uint8_t>(n, from + CHUNK_SZ);
    doc["count"] = (uint8_t)(to - from);

    bool hasRelativeTs = false;
    bool hasEpochTs = false;

    for (uint8_t i = from; i < to; i++) {
      if (hist_ts_is_relative(ordered[i].ts)) {
        hasRelativeTs = true;
      } else {
        hasEpochTs = true;
      }

      JsonArray pt = points.createNestedArray();
      pt.add(hist_ts_publish_value(ordered[i].ts));
      pt.add(ordered[i].temp);
    }

    doc["epoch_valid"] = hasEpochTs && !hasRelativeTs;
    doc["t_mode"] = (hasEpochTs && hasRelativeTs) ? "mixed" : (hasRelativeTs ? "relative_s" : "epoch_s");

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
