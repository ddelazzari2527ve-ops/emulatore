#ifndef MATH_HANDLING_H
#define MATH_HANDLING_H

#include <Arduino.h>
#include "system.h"
#include "table.h" // Il tuo file con l'array dei passi

// ==============================================================
//  PT1000 → Resistenza (Formula di Callendar-Van Dusen)
// ==============================================================
inline float tempAResistenza(float t) {
    return R0_PT1000 * (1.0f + CVD_A * t + CVD_B * t * t);
}

// ==============================================================
//  RICERCA RAPIDA NELLA TABELLA (Lookup binario)
// ==============================================================
inline bytes8_t trovaPassiPotenziometro(float r_target) {
    float r_pot = r_target - R_FISSA;

    if (r_pot <= table[0].r_tot)              return table[0];
    if (r_pot >= table[TAB_SIZE - 1].r_tot)   return table[TAB_SIZE - 1];

    int left = 0, right = TAB_SIZE - 1;
    while (left + 1 < right) {
        int mid = left + (right - left) / 2;
        if (table[mid].r_tot < r_pot) left = mid;
        else                          right = mid;
    }

    // Ritorna il valore più vicino tra i due trovati
    return (fabsf(table[left].r_tot - r_pot) <= fabsf(table[right].r_tot - r_pot))
           ? table[left] : table[right];
}

#endif