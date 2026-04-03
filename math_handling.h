#ifndef MATH_HANDLING_H
#define MATH_HANDLING_H
 
#include <Arduino.h>
#include <math.h>
#include "system.h"
#include "table.h"
 
// Converte la Temperatura in Resistenza PT1000 (Callendar-Van Dusen)
float calcolo_resistenza_ideale(float t) {
    return R0_PT1000 * (1.0f + CVD_A * t + CVD_B * t * t);
}
 
// Cerca nella tabella la coppia (p1,p2) con resistenza piu vicina al target
bytes8_t trova_potenziometri(float resistenza_totale_target) {
 
    float R_pot_target = resistenza_totale_target - R_FISSA;
 
    // Clamp fuori scala
    if (R_pot_target <= table[0].r_tot)
        return {table[0].p1, table[0].p2, table[0].r_tot};
    if (R_pot_target >= table[TAB_SIZE - 1].r_tot)
        return {table[TAB_SIZE-1].p1, table[TAB_SIZE-1].p2, table[TAB_SIZE-1].r_tot};
 
    // Ricerca binaria
    // FIX: rimosso confronto float con == (non scattava mai, sprecava cicli)
    int left  = 0;
    int right = TAB_SIZE - 1;
    while (left + 1 < right) {
        int mid = left + (right - left) / 2;
        if (table[mid].r_tot < R_pot_target)
            left  = mid;
        else
            right = mid;
    }
 
    // Scegli il piu vicino tra left e right
    float diffL = fabsf(table[left].r_tot  - R_pot_target);
    float diffR = fabsf(table[right].r_tot - R_pot_target);
 
    if (diffL <= diffR)
        return {table[left].p1,  table[left].p2,  table[left].r_tot};
    else
        return {table[right].p1, table[right].p2, table[right].r_tot};
}
 
#endif
 