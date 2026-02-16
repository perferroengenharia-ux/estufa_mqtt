#pragma once
#include <Arduino.h>

void  sensor_begin(uint8_t pinDQ, uint8_t resolutionBits);
void  sensor_update(unsigned long nowMs);

bool  sensor_ok();          // sensor detectado (endereço encontrado)
bool  sensor_has_value();   // temperatura válida disponível
float sensor_get_c();       // última temperatura (°C)
