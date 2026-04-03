#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>

// ==============================================================
//  STRUTTURA DATI PER MCP4261 (16 BIT)
// ==============================================================
struct bytes8_t {
    uint16_t p1;     
    uint16_t p2;     
    float    r_tot;  
};

// ==============================================================
//  PIN ESP32
// ==============================================================
#define PIN_CALORE_IN       35
#define PIN_RAFFREDDAMENTO   4
#define CS_MANDATA           5
#define CS_RITORNO          25

// ==============================================================
//  PT1000 E CALIBRAZIONE ADC
// ==============================================================
const float R_FISSA          = 1004.55f;
const float R0_PT1000        = 1000.0f;
const float CVD_A            = 3.9083e-3f;
const float CVD_B            = -5.775e-7f;

const int   ADC_ZERO_SOGLIA  = 50;       // Taglia il rumore analogico basso
const float ADC_CALDO_MAX    = 2164.0f;  // Valore letto dall'ADC quando riscalda al 100%
const float ADC_ALPHA        = 0.15f;    // Filtro pulizia segnale (0.01 - 1.00)

#endif