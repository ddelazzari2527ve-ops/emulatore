#include <Arduino.h>
#include <SPI.h>
#include "system.h"
#include "math_handling.h"

// ═══════════════════════════════════════════════════════════════
//  PIN
// ═══════════════════════════════════════════════════════════════
#define PIN_CALDO         35   // ADC  — CN-106 riscaldamento 0-24V
#define PIN_FREDDO         4   // DIG  — CN-105 raffreddamento (relè invertito)
#define PIN_POT_RITORNO   33   // ADC  — potenziometro temp. ritorno (T rete)

#define CS_USCITA          5   // MCP4261 #1 → BT52-1 (temp uscita) → CN-114
#define CS_RITORNO        25   // MCP4261 #2 → BT51-1 (temp ritorno) → CN-215

// ── FISICA ────────────────────────────────────────────────────
// Tutte le costanti derivano dalla fisica reale dell'ETP HT 06/12
// e vengono SCALATE a run-time con i valori inseriti dal Serial

#define CP_ACQUA         4186.0f   // J/kg°C
#define P_RISC_MAX      12000.0f   // W  (12 kW installati)

// Dispersione tubi: FISSA — non inserita dall'utente
// Tubi non isolati, circuito ~0.5m², U≈10W/m²K → ~5W/°K di scarto
// Modellata come Newton: P_disp = K_tubi * (T - T_amb)
// K_tubi = 5 W/K → per 5kg acqua: k_s = 5/(5*4186) = 0.000239 /s
#define K_DISPERSIONE_TUBI  0.000239f   // 1/s — NON modificabile
#define TEMP_AMBIENTE        20.0f
#define TEMP_MAX            400.0f

// Inerzia sonda PT1000 (filtra il segnale verso il PID del termoreg)
#define VELOCITA_SONDA      0.15f       // 0=sonda infinitamente lenta, 1=istantanea

// Range potenziometro ritorno (= temperatura acqua di rete)
#define T_RIT_MIN           5.0f
#define T_RIT_MAX          60.0f

// ── PARAMETRI INSERITI DAL SERIAL ────────────────────────────
// Valorizzati con default fisicamente coerenti
float portata_lmin    = 10.0f;   // L/min — range tipico 2-25
float massa_acqua_kg  =  5.0f;   // kg acqua nel circuito (fisso interno)

// Derivati da portata (ricalcolati ogni volta che portata cambia)
float portata_kgs     = 0.0f;    // kg/s
float risc_rate       = 0.0f;    // C/s a potenza 100%
float raff_base       = 0.0f;    // C/s con ΔT=80K sullo scambiatore

// ── CAMPIONAMENTO ─────────────────────────────────────────────
const unsigned long INTERVALLO_MS = 100;
#define N_CN106  8
#define N_POT   16
#define ADC_ZERO 50
int buf_cn106[N_CN106] = {0}; int idx_cn106 = 0;
int buf_pot[N_POT]     = {0}; int idx_pot   = 0;

// ── STATO ─────────────────────────────────────────────────────
float         temp_acqua   = TEMPERATURA_INIZIALE;
float         temp_sonda   = TEMPERATURA_INIZIALE;
float         temp_ritorno = T_RIT_MIN;
unsigned long t_last       = 0;
unsigned long t_serial     = 0;

// ── FUNZIONI CALCOLO ──────────────────────────────────────────

void aggiorna_parametri() {
    portata_kgs = portata_lmin / 60.0f;
    risc_rate   = P_RISC_MAX / (massa_acqua_kg * CP_ACQUA);
    raff_base   = portata_kgs * CP_ACQUA * 80.0f / (massa_acqua_kg * CP_ACQUA);
}

// Potenza riscaldante istantanea [W]
float pot_riscaldante(float frac_caldo) {
    return frac_caldo * P_RISC_MAX;
}

// Potenza raffreddamento scambiatore [W] — Q = m_dot * Cp * ΔT
float pot_raffreddamento(bool freddo_on, float T_usc, float T_rete) {
    if (!freddo_on || T_usc <= T_rete) return 0.0f;
    return portata_kgs * CP_ACQUA * (T_usc - T_rete);
}

// Potenza dispersione tubi [W] — Newton: P = K * (T-Tamb)
float pot_dispersione_tubi(float T_usc) {
    float dT = T_usc - TEMP_AMBIENTE;
    if (dT <= 0.0f) return 0.0f;
    // K_tubi = K_DISPERSIONE_TUBI * massa * Cp [W/K]
    float K_tubi_W = K_DISPERSIONE_TUBI * massa_acqua_kg * CP_ACQUA;
    return K_tubi_W * dT;
}

// Potenza scambio stampo [W] — bilancio: Q_in - Q_out - Q_disp = Q_stampo
// Q_stampo = Q_risc - Q_raff - Q_disp (positivo = cede calore allo stampo)
float pot_scambio_stampo(float Q_risc, float Q_raff, float Q_disp) {
    return Q_risc - Q_raff - Q_disp;
}

// ── SPI ───────────────────────────────────────────────────────
void scriviWiper(uint8_t cs, uint16_t v0, uint16_t v1) {
    byte c0 = 0x00; if (v0 > 255) c0 |= 0x01;
    byte c1 = 0x10; if (v1 > 255) c1 |= 0x01;
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    SPI.transfer(c0); SPI.transfer(v0 & 0xFF);
    SPI.transfer(c1); SPI.transfer(v1 & 0xFF);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

float leggi_caldo() {
    buf_cn106[idx_cn106] = analogRead(PIN_CALDO);
    idx_cn106 = (idx_cn106 + 1) % N_CN106;
    long s = 0; for (int i=0; i<N_CN106; i++) s += buf_cn106[i];
    int adc = (int)(s / N_CN106);
    if (adc < ADC_ZERO) return 0.0f;
    float f = (float)adc * (3.9f * 7.8f / (4095.0f * 24.0f));
    return (f > 1.0f) ? 1.0f : f;
}

float leggi_ritorno() {
    buf_pot[idx_pot] = analogRead(PIN_POT_RITORNO);
    idx_pot = (idx_pot + 1) % N_POT;
    long s = 0; for (int i=0; i<N_POT; i++) s += buf_pot[i];
    int adc = (int)(s / N_POT);
    float t = T_RIT_MIN + (float)adc * (T_RIT_MAX - T_RIT_MIN) / 4095.0f;
    if (t < T_RIT_MIN) t = T_RIT_MIN;
    if (t > T_RIT_MAX) t = T_RIT_MAX;
    return t;
}

// ── GESTIONE SERIAL MONITOR ───────────────────────────────────
void stampa_menu() {
    Serial.println();
    Serial.println("╔═══════════════════════════════════════╗");
    Serial.println("║   CONFIGURAZIONE TERMOREGOLATORE      ║");
    Serial.println("╠═══════════════════════════════════════╣");
    Serial.print  ("║ Portata attuale     : ");
    Serial.print(portata_lmin, 1);
    Serial.println(" L/min         ║");
    Serial.println("╠═══════════════════════════════════════╣");
    Serial.println("║ Comandi:                              ║");
    Serial.println("║  P<valore>  → portata L/min (2-25)   ║");
    Serial.println("║  ?          → mostra questo menu     ║");
    Serial.println("╚═══════════════════════════════════════╝");
    Serial.println();
}

void gestisci_serial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    char tipo = toupper(cmd.charAt(0));
    float val = (cmd.length() > 1) ? cmd.substring(1).toFloat() : 0.0f;

    if (tipo == 'P') {
        if (val >= 2.0f && val <= 25.0f) {
            portata_lmin = val;
            aggiorna_parametri();
            Serial.print(">> Portata impostata: ");
            Serial.print(portata_lmin, 1);
            Serial.println(" L/min");
        } else {
            Serial.println(">> Errore: portata deve essere 2-25 L/min");
        }
    } else if (tipo == '?') {
        stampa_menu();
    } else {
        Serial.println(">> Comando non riconosciuto. Digita ? per aiuto.");
    }
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);
    pinMode(PIN_FREDDO, INPUT_PULLUP);
    pinMode(CS_USCITA,  OUTPUT); digitalWrite(CS_USCITA,  HIGH);
    pinMode(CS_RITORNO, OUTPUT); digitalWrite(CS_RITORNO, HIGH);
    SPI.begin();

    aggiorna_parametri();
    stampa_menu();

    Serial.println("T_usc  | T_rend | T_rete | Caldo% | Freddo | P_risc  | P_raff  | P_disp | P_stampo");
    Serial.println("-------|--------|--------|--------|--------|---------|---------|--------|----------");
}

// ── LOOP ──────────────────────────────────────────────────────
void loop() {
    gestisci_serial();

    unsigned long now = millis();
    if (now - t_last < INTERVALLO_MS) return;
    float dt = (float)(now - t_last) * 0.001f;
    t_last = now;

    // ── Lettura segnali ───────────────────────────────────────
    float frac_caldo = leggi_caldo();
    bool  freddo_on  = (digitalRead(PIN_FREDDO) == LOW);
    temp_ritorno     = leggi_ritorno();   // = temperatura acqua di rete

    // ── Calcolo potenze ───────────────────────────────────────
    float Q_risc  = pot_riscaldante(frac_caldo);
    float Q_raff  = pot_raffreddamento(freddo_on, temp_acqua, temp_ritorno);
    float Q_disp  = pot_dispersione_tubi(temp_acqua);
    float Q_stamp = pot_scambio_stampo(Q_risc, Q_raff, Q_disp);

    // ── Fisica dell'acqua ─────────────────────────────────────

    // Riscaldamento proporzionale alla potenza CN-106
    temp_acqua += risc_rate * frac_caldo * dt;

    // Raffreddamento scambiatore (proporzionale a ΔT con rete)
    if (freddo_on && temp_acqua > temp_ritorno) {
        float deltaT   = temp_acqua - temp_ritorno;
        float rate_eff = raff_base * deltaT / 80.0f;
        temp_acqua -= rate_eff * dt;
    }

    // Dispersione tubi (Newton, valore fisso non parametrizzabile)
    float disp = K_DISPERSIONE_TUBI * (temp_acqua - TEMP_AMBIENTE) * dt;
    if (disp > 0.0f) temp_acqua -= disp;

    // Limiti fisici
    if (freddo_on && temp_acqua < temp_ritorno) temp_acqua = temp_ritorno;
    if (temp_acqua < TEMP_AMBIENTE) temp_acqua = TEMP_AMBIENTE;
    if (temp_acqua > TEMP_MAX)      temp_acqua = TEMP_MAX;

    // ── Inerzia sonda ─────────────────────────────────────────
    temp_sonda += (temp_acqua - temp_sonda) * VELOCITA_SONDA * dt;

    // ── Aggiorna MCP4261 ──────────────────────────────────────
    float    r_usc = calcolo_resistenza_ideale(temp_sonda);
    bytes8_t p_usc = trova_potenziometri(r_usc);
    scriviWiper(CS_USCITA, p_usc.p1, p_usc.p2);

    float    r_rit = calcolo_resistenza_ideale(temp_ritorno);
    bytes8_t p_rit = trova_potenziometri(r_rit);
    scriviWiper(CS_RITORNO, p_rit.p1, p_rit.p2);

    // ── Serial ogni 1s ────────────────────────────────────────
    if (now - t_serial >= 1000) {
        t_serial = now;
        // Temperature
        Serial.print(temp_acqua,  1); Serial.print("C  | ");
        Serial.print(temp_sonda,  1); Serial.print("C  | ");
        Serial.print(temp_ritorno,1); Serial.print("C  | ");
        // Segnali
        Serial.print(frac_caldo*100.0f,0); Serial.print("%  | ");
        Serial.print(freddo_on ? "ON     | " : "off    | ");
        // Potenze [W]
        Serial.print(Q_risc,  0); Serial.print("W  | ");
        Serial.print(Q_raff,  0); Serial.print("W  | ");
        Serial.print(Q_disp,  0); Serial.print("W  | ");
        Serial.print(Q_stamp, 0); Serial.println("W");
    }
}
