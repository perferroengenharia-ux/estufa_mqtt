#include "controlador_caap.h"
#include "mat.h"

// --- DEFINIÇÕES DE SEGURANÇA ---
#define A1_MIN 0.80f   // Sistema térmico é lento (polo perto de 1)
#define A1_MAX 0.999f
#define B0_MIN 0.0001f // Ganho mínimo
#define B0_MAX 0.5f    // Ganho máximo

// Limite de erro para resetar
#define ERRO_CRITICO 2.0f 
// Zona morta: se o erro for menor que isso, não atualiza RLS (evita drift)
#define DEAD_ZONE 0.15f 

void controlador_begin(CAAP_Data &data, float temp_inicial) {
    // Valores iniciais conservadores
    data.a1 = 0.99f;
    data.b0 = 0.0005f;
    
    // Reset da Matriz P (Reset de confiança)
    data.P[0][0] = 1000.0f; data.P[0][1] = 0.0f;
    data.P[1][0] = 0.0f;    data.P[1][1] = 1000.0f;
    
    data.temperatura_ant = temp_inicial;
    data.u_ant = 0.0f;
    data.u_calculado = 0.0f;
    
    data.lambda = 0.992f;
    data.polo_desejado = -0.8187f;
}

void controlador_update(CAAP_Data &data, float temp_atual, float setpoint) {
    // 1. Vetor de regressão
    float phi[2] = { data.temperatura_ant, data.u_ant };

    // 2. Predição a priori
    float y_hat = (data.a1 * phi[0]) + (data.b0 * phi[1]);
    float erro_predicao = temp_atual - y_hat;
    float erro_tracking = setpoint - temp_atual;

    // === LÓGICA DO SUPERVISOR (SEU PEDIDO) ===
    // Se o erro for grande (> 3C) E os parâmetros estiverem travados nos limites
    // Significa que o modelo matemático não condiz mais com a realidade.
    bool params_nos_limites = (data.a1 <= (A1_MIN + 0.01f)) || (data.b0 >= (B0_MAX - 0.01f));
    
    if (fabsf(erro_tracking) > ERRO_CRITICO && params_nos_limites) {
        // RESET SUAVE: Reinicia os parâmetros para valores seguros sem reiniciar o ESP
        data.a1 = 0.99f;
        data.b0 = 0.0005f;
        // Reseta a covariância para permitir aprendizado rápido novamente
        data.P[0][0] = 1000.0f; data.P[1][1] = 1000.0f;
        data.P[0][1] = data.P[1][0] = 0.0f;
        
        // Retorna para evitar atualizar com lixo nesta iteração
        return; 
    }

    // === ZONA MORTA (ANTI-DRIFT) ===
    // Só roda o RLS se houver algo relevante para aprender.
    // Se o erro de predição for minúsculo (ruído), ignoramos.
    bool deve_atualizar_rls = (fabsf(erro_predicao) > DEAD_ZONE);

    if (deve_atualizar_rls) {
        // 3. Ganho de Kalman
        float Pphi[2];
        Pphi[0] = (data.P[0][0] * phi[0]) + (data.P[0][1] * phi[1]);
        Pphi[1] = (data.P[1][0] * phi[0]) + (data.P[1][1] * phi[1]);
        
        float denom = data.lambda + (phi[0] * Pphi[0]) + (phi[1] * Pphi[1]);
        float K[2] = { Pphi[0] / denom, Pphi[1] / denom };

        // 4. Atualização dos Parâmetros
        data.a1 += K[0] * erro_predicao;
        data.b0 += K[1] * erro_predicao;

        // --- CLAMPING (Travas) ---
        if (data.a1 > A1_MAX) data.a1 = A1_MAX;
        if (data.a1 < A1_MIN) data.a1 = A1_MIN;
        
        if (data.b0 > B0_MAX) data.b0 = B0_MAX;
        if (data.b0 < B0_MIN) data.b0 = B0_MIN;

        // 5. Atualização da Matriz P
        float K_phiT_P[2][2];
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                K_phiT_P[i][j] = K[i] * (phi[0] * data.P[0][j] + phi[1] * data.P[1][j]);
            }
        }

        float inv_lambda = 1.0f / data.lambda;
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                data.P[i][j] = inv_lambda * (data.P[i][j] - K_phiT_P[i][j]);
            }
        }
    }

    // 6. Lei de Controle (Alocação de Polos)
    // Usamos os parâmetros (seja os aprendidos, travados ou resetados)
    float g0 = (data.polo_desejado + data.a1) / data.b0;
    float h0 = (1.0f + data.polo_desejado) / data.b0;

    float u = (h0 * setpoint) - (g0 * temp_atual);

    // 7. Saturação
    if (u > 100.0f) u = 100.0f;
    if (u < 0.0f)   u = 0.0f;

    data.u_calculado = u;
    
    // Atualiza histórico
    data.temperatura_ant = temp_atual;
    data.u_ant = u;
}

void controlador_apply_output(const CAAP_Data &data, uint8_t pin_ssr, unsigned long janela_ms) {
    static unsigned long inicio_janela = 0;
    unsigned long agora = millis();
    
    if (agora - inicio_janela >= janela_ms) {
        inicio_janela = agora;
    }

    unsigned long tempo_on = (unsigned long)((data.u_calculado / 100.0f) * janela_ms);

    if ((agora - inicio_janela) < tempo_on) {
        digitalWrite(pin_ssr, HIGH);
    } else {
        digitalWrite(pin_ssr, LOW);
    }
}