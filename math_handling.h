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
//  Modello fisico: Q = portata_kgs × CP_ACQUA × (T_serb − T_rete)
//
//  La potenza è proporzionale a ΔT → il serbatoio decade esponenzialmente
//  verso T_rete con costante di tempo τ = m_acqua / portata_kgs.
//  Quando T_serb → T_rete il flusso si azzera da solo: nessun clamp artificiale.
//
//  portata_kgs = p_portata_valvola [L/min] / 60   (convertita in kg/s)
//
//  Questa funzione è usata per:
//    • raffreddamento normale (hardware PIN o comando RAFFREDDAMENTO)
//    • iniezione ACQUA (stessa valvola, 3 secondi)
//  Per PERDITA si usa una portata fissa piccola (PORTATA_PERDITA_LMIN).
//
inline float potenzaRaffreddamento(bool  valvola_aperta,
                                   float t_serbatoio,
                                   float t_rete,
                                   float portata_kgs)
{
    if (!valvola_aperta || t_serbatoio <= t_rete + 0.05f) return 0.0f;
    return portata_kgs * CP_ACQUA * (t_serbatoio - t_rete);
}

// ── Dispersione termica nei tubi verso l'ambiente ────────────────────────────
inline float dispersioneTubi(float t_fluido) {
    float dT = t_fluido - p_temp_ambiente;
    return (dT > 0.0f) ? p_k_disp_tubi * dT : 0.0f;
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