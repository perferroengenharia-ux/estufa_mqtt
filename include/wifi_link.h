#pragma once
#include <Arduino.h>

void   wifi_begin();
void   wifi_update();

bool   wifi_is_connected();
String wifi_ip();
int    wifi_rssi();
