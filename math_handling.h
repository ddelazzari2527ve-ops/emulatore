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

// ── Potenza dissipativa dello Scambiatore di Calore (HX) ─────────────────────
//
//  Modello a conduttanza globale: Q = k_hx * (T_serb - T_rete)
//  Non vi è scambio di massa: l'acqua di rete rimane nel circuito secondario.
//  Vincolo fisico: se il serbatoio è già alla temperatura della rete (o più
//  freddo), la potenza estratta è zero — lo scambiatore non può pompare calore
//  verso la sorgente fredda.
//
//  @param attivo   true quando PIN_RAFFREDDAMENTO (raffreddamento_cmd) è attivo
//  @param t_serb   temperatura corrente del serbatoio [°C]
//  @param t_rete   temperatura dell'acqua di rete [°C]
//  @param k_hx     coefficiente di scambio termico [W/°C]
//  @return         potenza estratta [W], sempre >= 0
//
inline float potenzaScambiatore(bool  attivo,
                                float t_serb,
                                float t_rete,
                                float k_hx)
{
    if (!attivo || t_serb <= t_rete) return 0.0f;
    return k_hx * (t_serb - t_rete);
}

// ── Potenza di raffreddamento per iniezione diretta acqua di rete ─────────────
//
//  Modello fisico a bilancio di massa: Q = m_dot * Cp * (T_serb - T_rete)
//  Attivato da errore_acqua (comando ACQUA) o perdita_attiva (comando PERDITA).
//  L'acqua di rete entra fisicamente nel circuito, modificando la massa totale.
//
//  @param valvola_aperta  true quando il flusso di iniezione è > 0
//  @param t_serbatoio     temperatura corrente del serbatoio [°C]
//  @param t_rete          temperatura dell'acqua di rete [°C]
//  @param portata_kgs     portata massica di iniezione [kg/s]
//  @return                potenza estratta [W], sempre >= 0
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