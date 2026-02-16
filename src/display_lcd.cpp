#include "display_lcd.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

static LiquidCrystal_I2C* lcd = nullptr;
static uint8_t gCols = 16;
static uint8_t gRows = 2;

// Função auxiliar para garantir que a linha tenha exatamente 16 caracteres
static void printLine(uint8_t row, String text) {
  if (!lcd) return;
  lcd->setCursor(0, row);
  
  while (text.length() < gCols) {
    text += " "; 
  }
  if (text.length() > gCols) {
    text = text.substring(0, gCols);
  }
  
  lcd->print(text);
}

void display_begin(uint8_t addr, uint8_t cols, uint8_t rows) {
  gCols = cols;
  gRows = rows;
  lcd = new LiquidCrystal_I2C(addr, cols, rows);
  lcd->init();
  lcd->backlight();
  lcd->clear();
}

void display_show_boot(const char* line1, const char* line2) {
  if (!lcd) return;
  lcd->clear();
  printLine(0, String(line1));
  printLine(1, String(line2));
}

void display_update(bool systemOn, float setpoint, bool tempValid, float tempC, bool heaterOn) {
  if (!lcd) return;

  // --- LINHA 0: [T:30.0] esquerdo | [LIG/DESL] direito ---
  String tPart = "T:";
  tPart += tempValid ? String(tempC, 1) : "--.-";
  
  String sPart = systemOn ? "LIGADO" : "DESLIGADO";
  
  // Monta a linha 0 com espaços calculados para alinhar à direita
  String linha0 = tPart;
  int spaces0 = gCols - tPart.length() - sPart.length();
  for (int i = 0; i < spaces0; i++) linha0 += " ";
  linha0 += sPart;

  // --- LINHA 1: [SET:32.0] esquerdo | [AQ:ON/OFF] direito ---
  String setPart = "SET:" + String(setpoint, 1);
  
  String aqPart = "AQ:";
  aqPart += heaterOn ? "ON" : "OFF";
  
  // Monta a linha 1 com espaços calculados para alinhar à direita
  String linha1 = setPart;
  int spaces1 = gCols - setPart.length() - aqPart.length();
  for (int i = 0; i < spaces1; i++) linha1 += " ";
  linha1 += aqPart;

  // Envia para o display
  printLine(0, linha0);
  printLine(1, linha1);
}