#include <Arduino.h>

#include "sensor_ds18b20.h"
#include "buttons.h"
#include "display_lcd.h"
#include "controlador_caap.h"

#include "config.h"
#include "wifi_link.h"
#include "mqtt_link.h"

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

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ======= MQTT CMD HANDLER (roda na task de rede via mqtt.loop()) =======
static void on_mqtt_cmd(const MqttCommand& c) {
  // NÃO zere potência/sistema por falta de internet.
  // Só altera quando recebe comando válido.

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

  for (;;) {
    const uint32_t now = millis();

    wifi_update();
    mqtt_update();

    // Publica state periodicamente se MQTT estiver conectado
    if (mqtt_is_connected() && (now - lastPub >= MQTT_STATE_PUB_MS)) {
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
      s.id       = CTRL_ID;
      s.systemOn = localOn;
      s.heating  = heating;
      s.tempValid= tempValid;
      s.tempC    = tempC;
      s.setpoint = localSp;
      s.u_pct    = u_pct;
      s.a1       = meuControle.a1;
      s.b0       = meuControle.b0;
      s.rssi     = wifi_rssi();
      s.ms       = now;

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

  // SSR
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);

  // Módulos do processo (sempre locais)
  buttons_begin(PIN_BTN_ONOFF, PIN_BTN_UP, PIN_BTN_DOWN);

  display_begin(LCD_ADDR, LCD_COLS, LCD_ROWS);
  display_show_boot("ESTUFA MQTT", CTRL_ID);

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

void loop() {
  // loop vazio: tudo roda nas tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
