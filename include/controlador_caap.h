#pragma once
#include <Arduino.h>

// Estrutura para manter o estado do controlador adaptativo
struct CAAP_Data {
    // Parâmetros estimados
    float a1;
    float b0;
    
    // Matriz de covariância P (2x2)
    float P[2][2];
    
    // Histórico para o regressor
    float temperatura_ant;
    float u_ant;
    
    // Configurações
    float lambda;       // Fator de esquecimento
    float polo_desejado;
    float u_calculado;  // Saída 0-100%
};

// Inicializa os parâmetros do controlador
void controlador_begin(CAAP_Data &data, float temp_inicial);

// Executa a identificação RLS e calcula a nova lei de controle (deve rodar a cada 1s)
void controlador_update(CAAP_Data &data, float temp_atual, float setpoint);

// Gera o sinal PWM de baixa frequência para o SSR (deve rodar em todo loop)
void controlador_apply_output(const CAAP_Data &data, uint8_t pin_ssr, unsigned long janela_ms);