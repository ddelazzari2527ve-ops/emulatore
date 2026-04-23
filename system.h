#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════
//  STRUTTURA DATI
// ══════════════════════════════════════════════════════════════════
struct bytes8_t {
    uint16_t p1;
    uint16_t p2;
    float    r_tot;
};

// ══════════════════════════════════════════════════════════════════
//  PIN
// ══════════════════════════════════════════════════════════════════
#define PIN_CALORE_IN        35
#define PIN_RAFFREDDAMENTO    4
#define CS_MANDATA            5
#define CS_RITORNO           25
#define PIN_RELE_ACQUA       27

// ══════════════════════════════════════════════════════════════════
//  COSTANTI FISICHE
// ══════════════════════════════════════════════════════════════════
const float CP_ACQUA         = 4186.0f;
const float CP_ACCIAIO       = 500.0f;
const float K_STAMPO_DEFAULT = 200.0f;

// ══════════════════════════════════════════════════════════════════
//  PT1000 — Callendar-Van Dusen
// ══════════════════════════════════════════════════════════════════
const float R_FISSA   = 1004.55f;
const float R0_PT1000 = 1000.0f;
const float CVD_A     = 3.9083e-3f;
const float CVD_B     = -5.775e-7f;

// ══════════════════════════════════════════════════════════════════
//  ADC / SONDE
// ══════════════════════════════════════════════════════════════════
const int   ADC_ZERO_SOGLIA = 50;
const float ADC_CALDO_MAX   = 2145.0f;
const float ADC_ALPHA       = 0.15f;
const float DEADBAND_CALDO  = 0.015f;
const float ALPHA_SONDA     = 1.0f;  // Ricalibrato per tau ~ 3s (inerzia pozzetto)
const float OFFSET_MANDATA  = 0.45f;
const float OFFSET_RITORNO  = -0.45f;

// ══════════════════════════════════════════════════════════════════
//  TIMING
// ══════════════════════════════════════════════════════════════════
const unsigned long CICLO_MS          = 100;
const float         DT_MAX_S          = 0.30f;
const float         DEADTIME_S        = 2.0f;
const unsigned int  SENSOR_DT_MS      = 5;
const unsigned long TIMEOUT_RAFF_CONF = 5000UL;

// ══════════════════════════════════════════════════════════════════
//  DIGITAL TWIN HEATER
// ══════════════════════════════════════════════════════════════════
const float MASSA_RES_KG         = 0.5f;
const float K_SCAMBIO_RES        = 150.0f;
const float K_NUCLEO             = 300.0f;
const float M_NUCLEO             = 2.8f;
const float PORTATA_PERDITA_LMIN = 0.5f;

// ══════════════════════════════════════════════════════════════════
//  DEAD-TIME BUFFER — dimensione calcolata dalle costanti di timing
// ══════════════════════════════════════════════════════════════════
static const int DEAD_BUF_SIZE = (int)(DEADTIME_S / (CICLO_MS * 0.001f) + 0.5f);

// ══════════════════════════════════════════════════════════════════
//  PARAMETRI CONFIGURABILI — definiti in EFP.ino
// ══════════════════════════════════════════════════════════════════
extern float p_temp_rete;
extern float p_portata;
extern float p_massa_acqua;
extern float p_pot_risc;
extern float p_k_hx;           // [W/°C] Coefficiente scambio termico HX (ex p_pot_raff)
extern float p_pot_stampo;
extern float p_temp_ambiente;
extern float p_k_disp_tubi;
extern float p_k_disp_serbatoio;
extern float p_massa_stampo;
extern float K_stampo;
extern float ff_gain;
extern float p_portata_valvola; // [L/min] Solo per iniezione diretta acqua di rete

// ══════════════════════════════════════════════════════════════════
//  STATO TERMODINAMICO — definito in EFP.ino
// ══════════════════════════════════════════════════════════════════
extern float temp_resistenza;
extern float temp_nucleo_res;
extern float temp_serbatoio;
extern float temp_sonda_mandata;
extern float temp_sonda_ritorno;
extern float temp_stampo;
extern float adc_filtrato;

// ══════════════════════════════════════════════════════════════════
//  DEAD-TIME BUFFER — definito in EFP.ino
// ══════════════════════════════════════════════════════════════════
extern float dead_buffer[];
extern int   dead_buf_idx;

// ══════════════════════════════════════════════════════════════════
//  FLAG DI CONTROLLO — definiti in EFP.ino
// ══════════════════════════════════════════════════════════════════
extern bool          errore_acqua;
extern bool          in_attesa_acqua;
extern unsigned long t_attesa_acqua;
extern bool          raffreddamento_cmd;
extern bool          raffreddo_cmd_pendente;
extern unsigned long t_raff_cmd;
extern bool          perdita_attiva;

// ══════════════════════════════════════════════════════════════════
//  TIMING LOOP — definiti in EFP.ino
// ══════════════════════════════════════════════════════════════════
extern unsigned long t_last;
extern unsigned long t_log;

#endif