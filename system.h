#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>

struct bytes8_t {
    uint16_t p1;
    uint16_t p2;
    float    r_tot;
};

// PIN
#define PIN_CALORE_IN       35
#define PIN_RAFFREDDAMENTO   4
#define CS_MANDATA           5
#define CS_RITORNO          25

// COSTANTI FISICHE
const float CP_ACQUA         = 4186.0f;
const float CP_ACCIAIO       = 500.0f;
const float K_STAMPO_DEFAULT = 200.0f;

// PT1000
const float R_FISSA          = 1004.55f;
const float R0_PT1000        = 1000.0f;
const float CVD_A            = 3.9083e-3f;
const float CVD_B            = -5.775e-7f;

// ADC / SONDE / TIMING
const int   ADC_ZERO_SOGLIA  = 50;
const float ADC_CALDO_MAX    = 2164.0f;
const float ADC_ALPHA        = 0.15f;
const float DEADBAND_CALDO   = 0.015f;
const float ALPHA_SONDA      = 2.0f;
const float OFFSET_MANDATA   = 0.41f;

const unsigned long CICLO_MS = 100;
const float DT_MAX_S         = 0.30f;
const float DEADTIME_S       = 4.0f;
const unsigned int SENSOR_DT_MS = 5;

// DIGITAL TWIN HEATER
const float MASSA_RES_KG     = 0.5f;
const float K_SCAMBIO_RES    = 150.0f;
const float K_NUCLEO         = 300.0f;
const float M_NUCLEO         = 2.8f;

// PARAMETRI CONFIGURABILI
extern float p_temp_rete;
extern float p_portata;
extern float p_massa_acqua;
extern float p_pot_risc;
extern float p_pot_raff;
extern float p_pot_stampo;

// NUOVE VARIABILI
extern float p_temp_ambiente;
extern float p_k_disp_tubi;
extern float p_k_disp_serbatoio;
extern float p_massa_stampo;
extern float K_stampo;
extern float temp_stampo;
extern float stampo_ramp_rate;
extern float ff_gain;

#endif