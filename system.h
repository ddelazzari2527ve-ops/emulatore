#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════
//  STRUTTURA DATI PER LA TABELLA DEI POTENZIOMETRI DIGITALI
// ══════════════════════════════════════════════════════════════════
struct bytes8_t {
    uint16_t p1;
    uint16_t p2;
    float    r_tot;
};

// ══════════════════════════════════════════════════════════════════
//  MODALITÀ POMPA
// ══════════════════════════════════════════════════════════════════
enum PumpMode : uint8_t {
    PUMP_MODE_AUTO = 0,
    PUMP_MODE_MANUAL_OFF,
    PUMP_MODE_MANUAL_ON
};

// ══════════════════════════════════════════════════════════════════
//  PINOUT
// ══════════════════════════════════════════════════════════════════
static constexpr uint8_t PIN_CALORE_IN      = 35;
static constexpr uint8_t PIN_RAFFREDDAMENTO  = 4;
static constexpr uint8_t CS_MANDATA         = 5;
static constexpr uint8_t CS_RITORNO         = 25;
static constexpr uint8_t PIN_RELE_ACQUA     = 27;

// ══════════════════════════════════════════════════════════════════
//  COSTANTI TERMODINAMICHE E DI MATERIALI
// ══════════════════════════════════════════════════════════════════
static constexpr float CP_ACQUA               = 4186.0f;   // J/(kg·K)
static constexpr float R_FISSA                = 1004.55f;  // Ω, resistenza fissa di front-end
static constexpr float CP_ACCIAIO             = 500.0f;    // J/(kg·K)
static constexpr float CP_OLIO_VAL            = 2100.0f;   // J/(kg·K)
static constexpr float K_STAMPO_DEFAULT       = 200.0f;    // W/K
static constexpr float MASSA_RES_KG           = 0.5f;      // massa equivalente resistenza
static constexpr float K_SCAMBIO_RES          = 150.0f;    // W/K tra resistenza e nucleo
static constexpr float K_NUCLEO               = 300.0f;    // W/K tra nucleo e serbatoio
static constexpr float M_NUCLEO               = 2.8f;      // kg equivalenti del nucleo
static constexpr float K_RES_DRY              = 18.0f;     // W/K: perdita verso ambiente in secco
static constexpr float MASSA_ACQUA_NOMINALE   = 15.0f;     // kg, valore di fabbrica
static constexpr float MASSA_CRITICA_ACQUA    = 2.0f;      // kg, soglia allarme dry-run
static constexpr float MASSA_TERMICA_MIN      = 0.25f;     // kg equivalenti minimi
static constexpr float MASSA_SERBATOIO_VUOTO_ON_KG  = 1.0f; // kg, attiva rifornimento
static constexpr float MASSA_SERBATOIO_VUOTO_OFF_KG = 4.0f; // kg, disattiva rifornimento
static constexpr float PORTATA_NOMINALE_LMIN  = 20.0f;     // L/min, riferimento nominale
static constexpr float PORTATA_MIN_CIRC_LMIN   = 1.0f;      // L/min, sotto questa soglia la circolazione è insufficiente
static constexpr float PORTATA_PERDITA_LMIN    = 0.5f;      // L/min equivalente leak/recovery
static constexpr float PORTATA_VALVOLA_MAX_LMIN = 20.0f;    // L/min, limite comando rifornimento
static constexpr float PUMP_HEAT_W             = 200.0f;    // W immessi dalla pompa nel fluido
static constexpr float THERMAL_SHOCK_DELTA_C    = 40.0f;    // °C
static constexpr float POTENZA_RISCALDO_MAX_W   = 25000.0f; // W
static constexpr float POTENZA_RAFFREDDAMENTO_MAX_W = 8000.0f; // W

// ══════════════════════════════════════════════════════════════════
//  PT1000 — CALLENDAR-VAN DUSEN
// ══════════════════════════════════════════════════════════════════
static constexpr float R0_PT1000                = 1000.0f;
static constexpr float CVD_A                    = 3.9083e-3f;
static constexpr float CVD_B                    = -5.775e-7f;
static constexpr float CVD_C                    = -4.183e-12f;
static constexpr float PT1000_MIN_TEMP_C        = -200.0f;
static constexpr float PT1000_MAX_TEMP_C        = 850.0f;
static constexpr float PT1000_SAFE_MIN_RES_OHM  = 18.0f;
static constexpr float PT1000_SAFE_MAX_RES_OHM  = 5000.0f;
static constexpr float R_OFFSET_MANDATA_OHM     = 5.77f;   // offset hardware centralizzato
static constexpr float R_OFFSET_RITORNO_OHM     = -1.92f;  // offset hardware centralizzato

// ══════════════════════════════════════════════════════════════════
//  ADC / SONDE / FILTRI
// ══════════════════════════════════════════════════════════════════
static constexpr int   ADC_ZERO_SOGLIA  = 50;
static constexpr float ADC_CALDO_MAX    = 2145.0f;
static constexpr float ADC_ALPHA         = 0.15f;     // filtro ingresso caldo
static constexpr float HEAT_ON_TH       = 0.030f;     // isteresi ON
static constexpr float HEAT_OFF_TH      = 0.015f;     // isteresi OFF
static constexpr float SONDA_TAU_S      = 0.200f;     // τ ≈ 200 ms

// ══════════════════════════════════════════════════════════════════
//  TIMING REAL-TIME
// ══════════════════════════════════════════════════════════════════
static constexpr unsigned long CICLO_MS          = 100UL;
static constexpr float         CICLO_S           = 0.100f;
static constexpr float         DT_MAX_S          = 0.30f;
static constexpr float         DEADTIME_S        = 2.0f;
static constexpr float         DEADTIME_MAX_S    = 5.0f;
static constexpr unsigned long TIMEOUT_RAFF_CONF = 5000UL;

// Buffer dimensionato per coprire ritardi maggiorati a bassa portata.
static constexpr int DEAD_BUF_SIZE = 50;

// ══════════════════════════════════════════════════════════════════
//  PARAMETRI CONFIGURABILI — DEFINITI IN EFP.ino
// ══════════════════════════════════════════════════════════════════
extern float p_temp_rete;
extern float p_portata;
extern float p_massa_acqua;
extern float p_pot_risc;
extern float p_pot_raffreddamento;
extern float p_k_hx;
extern float p_pot_stampo;
extern float p_temp_ambiente;
extern float p_k_disp_tubi;
extern float p_k_disp_serbatoio;
extern float p_massa_stampo;
extern float K_stampo;
extern float ff_gain;
extern float p_portata_valvola;

// ══════════════════════════════════════════════════════════════════
//  STATO TERMICO — DEFINITO IN EFP.ino
// ══════════════════════════════════════════════════════════════════
extern float temp_resistenza;
extern float temp_nucleo_res;
extern float temp_serbatoio;
extern float temp_sonda_mandata;
extern float temp_sonda_ritorno;
extern float temp_stampo;
extern float adc_filtrato;
extern float t_mandata_stasi;
extern float t_ritorno_stasi;
extern float t_mandata_reale;
extern float t_ritorno_reale;

// ══════════════════════════════════════════════════════════════════
//  DEAD-TIME BUFFER — DEFINITO IN EFP.ino
// ══════════════════════════════════════════════════════════════════
extern float dead_buffer[DEAD_BUF_SIZE];
extern int   dead_buf_idx;

// ══════════════════════════════════════════════════════════════════
//  FLAG DI CONTROLLO — DEFINITI IN EFP.ino
// ══════════════════════════════════════════════════════════════════
extern bool          errore_acqua;
extern bool          in_attesa_acqua;
extern unsigned long t_attesa_acqua;
extern bool          raffreddamento_cmd;
extern bool          perdita_attiva;
extern bool          raffreddamento_attivo;
extern bool          stato_pompa;
extern PumpMode      pump_mode;
extern bool          pump_manual_override;
extern bool          rele_carico_acqua;

// ══════════════════════════════════════════════════════════════════
//  ALLARMI — DEFINITI IN EFP.ino
// ══════════════════════════════════════════════════════════════════
extern bool alarm_boiling;
extern bool alarm_dry_run;
extern bool alarm_pump_fail;
extern bool warning_thermal_shock;
extern bool alarm_circulation_insufficient;
extern bool alarm_serbatoio_vuoto;

// ══════════════════════════════════════════════════════════════════
//  TIMING LOOP — DEFINITI IN EFP.ino
// ══════════════════════════════════════════════════════════════════
extern unsigned long t_last;
extern unsigned long t_log;

#endif
