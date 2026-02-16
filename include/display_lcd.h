#pragma once
#include <Arduino.h>

void display_begin(uint8_t addr, uint8_t cols, uint8_t rows);
void display_show_boot(const char* line1, const char* line2);

// Adicionado o parâmetro 'heaterOn' ao final
void display_update(bool systemOn, float setpoint, bool tempValid, float tempC, bool heaterOn);