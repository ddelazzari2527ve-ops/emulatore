#ifndef MATH_HANDLING_H
#define MATH_HANDLING_H

#include <Arduino.h>
#include <math.h>
#include "system.h"
#include "table.h"

inline float tempAResistenza(float t) {
    return R0_PT1000 * (1.0f + CVD_A * t + CVD_B * t * t);
}

inline float filtraPL(float y_prec, float x_nuovo, float alpha) {
    return y_prec + alpha * (x_nuovo - y_prec);
}

inline float potenzaRiscaldo(float fraz_caldo, float p_max_w) {
    return fraz_caldo * p_max_w;
}

inline float potenzaRaffreddamentoProporzionale(bool valvola_aperta,
                                                float t_serbatoio,
                                                float t_rete,
                                                float p_pot_raff_max,
                                                float massa_kg,
                                                float dt)
{
    if (!valvola_aperta || t_serbatoio <= t_rete) return 0.0f;
    float dT = t_serbatoio - t_rete;
    const float tau_eff = 5.0f;
    float eff = 1.0f - expf(-dT / tau_eff);
    eff = constrain(eff, 0.0f, 1.0f);
    float Q_raff = p_pot_raff_max * eff;
    float Q_max = dT * massa_kg * CP_ACQUA / dt;
    return min(Q_raff, Q_max);
}

inline float dispersioneNaturale(float t_serbatoio) {
    float dT = t_serbatoio - p_temp_ambiente;
    return (dT > 0.0f) ? p_k_disp_serbatoio * dT : 0.0f;
}

inline float dispersioneTubi(float t_fluido) {
    float dT = t_fluido - p_temp_ambiente;
    return (dT > 0.0f) ? p_k_disp_tubi * dT : 0.0f;
}

inline bytes8_t trovaPassiPotenziometro(float r_target) {
    float r_pot = r_target - R_FISSA;
    if (r_pot <= table[0].r_tot) return table[0];
    if (r_pot >= table[TAB_SIZE - 1].r_tot) return table[TAB_SIZE - 1];

    int left = 0, right = TAB_SIZE - 1;
    while (left + 1 < right) {
        int mid = left + (right - left) / 2;
        if (table[mid].r_tot < r_pot) left = mid;
        else right = mid;
    }
    return (fabsf(table[left].r_tot - r_pot) <= fabsf(table[right].r_tot - r_pot))
           ? table[left] : table[right];
}

#endif