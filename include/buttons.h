#pragma once
#include <Arduino.h>

enum BtnEvent { EV_NONE, EV_PRESS, EV_REPEAT };

void buttons_begin(uint8_t pinOnOff, uint8_t pinUp, uint8_t pinDown);
void buttons_update(unsigned long nowMs);

// eventos “latched” do último update
BtnEvent buttons_onoff_event();
BtnEvent buttons_up_event();
BtnEvent buttons_down_event();
