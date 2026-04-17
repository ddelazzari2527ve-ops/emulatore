#ifndef MATH_HANDLING_H
#define MATH_HANDLING_H

#include <Arduino.h>
#include <math.h>
#include "system.h"
#include "table.h"

// ── Resistenza PT1000 dalla temperatura (Callendar-Van Dusen) ────────────────
inline float tempAResistenza(float t) {
    return R0_PT1000 * (1.0f + CVD_A * t + CVD_B * t * t);
}

// ── Filtro passa-basso al primo ordine ───────────────────────────────────────
inline float filtraPL(float y_prec, float x_nuovo, float alpha) {
    return y_prec + alpha * (x_nuovo - y_prec);
}

// ── Potenza riscaldamento elettrico ──────────────────────────────────────────
inline float potenzaRiscaldo(float fraz_caldo, float p_max_w) {
    return fraz_caldo * p_max_w;
}

// ── Potenza di raffreddamento per iniezione diretta acqua di rete ─────────────
//
//  Modello fisico a bilancio di massa: Q = m_dot * Cp * (T_serb - T_rete)
//  Eliminati limiti arbitrari p_pot_raff. La potenza dipende solo dalla fisica.
//
inline float potenzaRaffreddamento(bool  valvola_aperta,
                                   float t_serbatoio,
                                   float t_rete,
                                   float portata_kgs)
{
    if (!valvola_aperta || t_serbatoio <= t_rete) return 0.0f;
    return portata_kgs * CP_ACQUA * (t_serbatoio - t_rete);
}

// ── Dispersione termica proporzionale al salto termico ────────────────────────
inline float dispersioneTubi(float t_fluido) {
    return p_k_disp_tubi * (t_fluido - p_temp_ambiente);
}

// ── Ricerca binaria nella tabella potenziometro ──────────────────────────────
inline bytes8_t trovaPassiPotenziometro(float r_target) {
    float r_pot = r_target - R_FISSA;
    if (r_pot <= table[0].r_tot)             return table[0];
    if (r_pot >= table[TAB_SIZE - 1].r_tot) return table[TAB_SIZE - 1];

    int left = 0, right = TAB_SIZE - 1;
    while (left + 1 < right) {
        int mid = left + (right - left) / 2;
        if (table[mid].r_tot < r_pot) left = mid;
        else                          right = mid;
    }
    return (fabsf(table[left].r_tot - r_pot) <= fabsf(table[right].r_tot - r_pot))
           ? table[left] : table[right];
}

#endif