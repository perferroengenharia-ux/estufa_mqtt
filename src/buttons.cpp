#include "buttons.h"

// Botões: INPUT_PULLUP (pressionado = LOW)

struct Button {
  uint8_t pin;
  unsigned long debounceMs;
  unsigned long repeatDelayMs;
  unsigned long repeatRateMs;

  int rawLast;
  int stable;
  unsigned long tChange;

  bool pressed;
  unsigned long pressedTime;
  unsigned long lastRepeat;

  void begin(uint8_t p, unsigned long deb=30, unsigned long repDelay=500, unsigned long repRate=150) {
    pin = p;
    debounceMs = deb;
    repeatDelayMs = repDelay;
    repeatRateMs = repRate;

    pinMode(pin, INPUT_PULLUP);
    rawLast = digitalRead(pin);
    stable = rawLast;
    tChange = millis();

    pressed = (stable == LOW);
    pressedTime = 0;
    lastRepeat = 0;
  }

  BtnEvent update(unsigned long now, bool allowRepeat) {
    int raw = digitalRead(pin);

    if (raw != rawLast) {
      rawLast = raw;
      tChange = now;
    }

    // debounce: estabilizou
    if ((now - tChange) >= debounceMs && raw != stable) {
      stable = raw;

      if (stable == LOW) { // pressionou
        pressed = true;
        pressedTime = now;
        lastRepeat = now;
        return EV_PRESS;
      } else {             // soltou
        pressed = false;
      }
    }

    // auto-repeat
    if (allowRepeat && pressed && stable == LOW) {
      if ((now - pressedTime) >= repeatDelayMs) {
        if ((now - lastRepeat) >= repeatRateMs) {
          lastRepeat = now;
          return EV_REPEAT;
        }
      }
    }

    return EV_NONE;
  }
};

static Button bOnOff, bUp, bDown;
static BtnEvent evOnOff = EV_NONE;
static BtnEvent evUp    = EV_NONE;
static BtnEvent evDown  = EV_NONE;

void buttons_begin(uint8_t pinOnOff, uint8_t pinUp, uint8_t pinDown) {
  bOnOff.begin(pinOnOff, 30, 500, 150);
  bUp.begin(pinUp, 30, 500, 150);
  bDown.begin(pinDown, 30, 500, 150);
}

void buttons_update(unsigned long nowMs) {
  // limpa eventos do ciclo anterior
  evOnOff = EV_NONE;
  evUp = EV_NONE;
  evDown = EV_NONE;

  // ON/OFF sem repeat
  evOnOff = bOnOff.update(nowMs, false);

  // UP/DOWN com repeat
  evUp   = bUp.update(nowMs, true);
  evDown = bDown.update(nowMs, true);
}

BtnEvent buttons_onoff_event() { return evOnOff; }
BtnEvent buttons_up_event()    { return evUp; }
BtnEvent buttons_down_event()  { return evDown; }
