#ifndef MATH_HANDLING_H
#define MATH_HANDLING_H

#include <Arduino.h>
#include <math.h>
#include "system.h"
#include "table.h"

// ══════════════════════════════════════════════════════════════════
//  UTILITÀ NUMERICHE
// ══════════════════════════════════════════════════════════════════
inline float clampFloat(float x, float lo, float hi) {
    return constrain(x, lo, hi);
}

inline float clampTempRealistica(float t) {
    return clampFloat(t, -20.0f, 150.0f);
}

// α = dt / (τ + dt)
inline float alphaDaTau(float dt_s, float tau_s) {
    if (dt_s <= 0.0f) return 0.0f;
    if (tau_s <= 0.0f) return 1.0f;
    float alpha = dt_s / (tau_s + dt_s);
    return clampFloat(alpha, 0.0f, 1.0f);
}

// ══════════════════════════════════════════════════════════════════
//  PT1000 FORWARD MODEL
// ══════════════════════════════════════════════════════════════════
inline float tempAResistenza(float t_c) {
    float t = clampFloat(t_c, PT1000_MIN_TEMP_C, PT1000_MAX_TEMP_C);
    float r = R0_PT1000 * (1.0f + CVD_A * t + CVD_B * t * t);
    if (t < 0.0f) {
        r += R0_PT1000 * CVD_C * (t - 100.0f) * t * t * t;
    }
    return max(r, PT1000_SAFE_MIN_RES_OHM);
}

inline float resistenzaPt1000Corretta(float t_c, float offset_ohm) {
    float r = tempAResistenza(t_c) + offset_ohm;
    return clampFloat(r, PT1000_SAFE_MIN_RES_OHM, PT1000_SAFE_MAX_RES_OHM);
}

// ══════════════════════════════════════════════════════════════════
//  PT1000 INVERSE MODEL
// ══════════════════════════════════════════════════════════════════
inline float tempDaResistenzaPt1000(float r_ohm) {
    float r = clampFloat(r_ohm, PT1000_SAFE_MIN_RES_OHM, PT1000_SAFE_MAX_RES_OHM);
    float t = (r / R0_PT1000 - 1.0f) / CVD_A;
    t = clampFloat(t, PT1000_MIN_TEMP_C, PT1000_MAX_TEMP_C);

    for (int i = 0; i < 8; ++i) {
        float f = 0.0f;
        float df = 0.0f;
        if (t >= 0.0f) {
            f  = R0_PT1000 * (1.0f + CVD_A * t + CVD_B * t * t) - r;
            df = R0_PT1000 * (CVD_A + 2.0f * CVD_B * t);
        } else {
            float t2 = t * t;
            float t3 = t2 * t;
            f  = R0_PT1000 * (1.0f + CVD_A * t + CVD_B * t2 + CVD_C * (t - 100.0f) * t3) - r;
            df = R0_PT1000 * (CVD_A + 2.0f * CVD_B * t + CVD_C * (4.0f * t3 - 300.0f * t2));
        }

        if (fabsf(df) < 1e-8f) break;
        t -= f / df;
        t = clampFloat(t, PT1000_MIN_TEMP_C, PT1000_MAX_TEMP_C);
    }

    return t;
}

// ══════════════════════════════════════════════════════════════════
//  BILANCI ENERGETICI
// ══════════════════════════════════════════════════════════════════
inline float filtraPL(float y_prec, float x_nuovo, float alpha) {
    return y_prec + alpha * (x_nuovo - y_prec);
}

inline float potenzaRiscaldo(float fraz_caldo, float p_max_w) {
    return fraz_caldo * p_max_w;
}

// Bidirezionale: positivo se il serbatoio cede calore alla rete,
// negativo se la rete cede calore al serbatoio.
inline float potenzaScambiatore(bool attivo,
                                float t_serb,
                                float t_rete,
                                float k_hx) {
    if (!attivo || k_hx <= 0.0f) return 0.0f;
    return k_hx * (t_serb - t_rete);
}

inline float potenzaRaffreddamento(bool attivo,
                                   float t_serbatoio,
                                   float t_rete,
                                   float portata_kgs) {
    if (!attivo || portata_kgs <= 0.0f) return 0.0f;
    float dT = t_serbatoio - t_rete;
    if (dT <= 0.0f) return 0.0f;
    return portata_kgs * CP_ACQUA * dT;
}

inline float dispersioneTubi(float t_fluido) {
    return p_k_disp_tubi * (t_fluido - p_temp_ambiente);
}

// ══════════════════════════════════════════════════════════════════
//  RICERCA SU TABELLA POTENZIOMETRO DIGITALE
// ══════════════════════════════════════════════════════════════════
inline bytes8_t trovaPassiPotenziometro(float r_target) {
    float r_pot = r_target - R_FISSA;
    if (r_pot <= table[0].r_tot)            return table[0];
    if (r_pot >= table[TAB_SIZE - 1].r_tot) return table[TAB_SIZE - 1];

    int left = 0;
    int right = TAB_SIZE - 1;
    while (left + 1 < right) {
        int mid = left + (right - left) / 2;
        if (table[mid].r_tot < r_pot) left = mid;
        else                          right = mid;
    }

    return (fabsf(table[left].r_tot - r_pot) <= fabsf(table[right].r_tot - r_pot))
             ? table[left]
             : table[right];
}

#endif
