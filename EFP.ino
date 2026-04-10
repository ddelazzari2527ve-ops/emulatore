#include <Arduino.h>
#include <SPI.h>
#include "system.h"
#include "math_handling.h"
#include "table.h"

// ==================== AGGIUNTA PIN RELÈ ====================
#define PIN_RELE_ACQUA 27

// ══════════════════════════════════════════════════════════════════
//  PARAMETRI CONFIGURABILI VIA SERIAL MONITOR E STATO
// ══════════════════════════════════════════════════════════════════
float p_temp_rete        = 15.0f;
float p_portata          = 20.0f;
float p_massa_acqua      = 15.0f;
float p_pot_risc         = 12000.0f;
float p_pot_raff         = 20000.0f;
float p_pot_stampo       = 0.0f;
float p_temp_ambiente    = 20.0f;
float p_k_disp_tubi      = 2.5f;
float p_k_disp_serbatoio = 5.0f;
float p_massa_stampo     = 15.0f;
float K_stampo           = K_STAMPO_DEFAULT;
float stampo_ramp_rate   = 2.0f;
float ff_gain            = 1.0f;

// Parametri Setpoint
float p_temp_stampo_set  = 20.0f;
bool  stampo_auto        = true;

// ══════════════════════════════════════════════════════════════════
//  STATO TERMODINAMICO E CONTROLLI
// ══════════════════════════════════════════════════════════════════
float temp_resistenza    = 15.0f;
float temp_nucleo_res    = 15.0f;
float temp_serbatoio     = 15.0f;
float temp_sonda_mandata = 15.0f;
float temp_sonda_ritorno = 15.0f;
float temp_stampo        = 20.0f;
float adc_filtrato       = 0.0f;

// Controlli Macchina
bool errore_acqua        = false;
bool interruttore_on     = true;
bool era_spenta          = false;

// --- VARIABILI PER IL RITARDO DI 3 SECONDI ---
bool in_attesa_acqua     = false;
unsigned long t_attesa_acqua = 0;

// Buffer per Dead-time
static const int DEAD_BUF_SIZE = (int)(DEADTIME_S / (CICLO_MS * 0.001f) + 0.5f);
float dead_buffer[DEAD_BUF_SIZE];
int   dead_buf_idx = 0;

float temp_stampo_set_attiva = 20.0f;

unsigned long t_last = 0;
unsigned long t_log  = 0;

// ══════════════════════════════════════════════════════════════════
//  FUNZIONI DI SUPPORTO
// ══════════════════════════════════════════════════════════════════
static inline float clampf_local(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void resetStatoTermico() {
    float t0 = p_temp_rete;
    temp_resistenza = temp_nucleo_res = t0;
    temp_serbatoio  = temp_sonda_mandata = temp_sonda_ritorno = t0;
    temp_stampo     = p_temp_ambiente;
    for (int i = 0; i < DEAD_BUF_SIZE; i++) dead_buffer[i] = t0;
    dead_buf_idx = 0;
    adc_filtrato = 0.0f;
    temp_stampo_set_attiva = p_temp_ambiente;
}

void inviaWiper(uint8_t cs, bytes8_t passi) {
    uint8_t cmd0 = 0x00 | (passi.p1 > 255 ? 0x01 : 0x00);
    uint8_t cmd1 = 0x10 | (passi.p2 > 255 ? 0x01 : 0x00);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    SPI.transfer(cmd0); SPI.transfer((uint8_t)(passi.p1 & 0xFF));
    SPI.transfer(cmd1); SPI.transfer((uint8_t)(passi.p2 & 0xFF));
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

float leggiComandoRiscaldo() {
    int raw = analogRead(PIN_CALORE_IN);
    if (raw < ADC_ZERO_SOGLIA) {
        adc_filtrato = filtraPL(adc_filtrato, 0.0f, ADC_ALPHA);
        return 0.0f;
    }
    float frac = constrain((float)raw / ADC_CALDO_MAX, 0.0f, 1.0f);
    adc_filtrato = filtraPL(adc_filtrato, frac, ADC_ALPHA);
    return (adc_filtrato < DEADBAND_CALDO) ? 0.0f : adc_filtrato;
}

// ══════════════════════════════════════════════════════════════════
//  SERIAL MONITOR
// ══════════════════════════════════════════════════════════════════
void stampaMenu() {
    Serial.println(F("\n========= TERMOREGOLATORE DIGITAL TWIN ========="));
    Serial.println(F("Comandi: T E L A R F S X Y M Z B ? | ON / OFF | ACQUA / PERDITA / RIPRISTINA"));
    Serial.println(F("-------------------------------------------------------"));
    Serial.printf("[T] Rete      : %.1f °C\n",    p_temp_rete);
    Serial.printf("[E] Ambiente  : %.1f °C\n",    p_temp_ambiente);
    Serial.printf("[L] Portata   : %.1f L/min\n", p_portata);
    Serial.printf("[A] Massa H2O : %.1f kg\n",    p_massa_acqua);
    Serial.printf("[M] Massa st. : %.1f kg\n",    p_massa_stampo);
    Serial.printf("[X] Disp.tubi : %.2f W/°C\n",  p_k_disp_tubi);
    Serial.printf("[Y] Disp.serb : %.2f W/°C\n",  p_k_disp_serbatoio);
    Serial.printf("[R] Pot.risc  : %.0f W\n",     p_pot_risc);
    Serial.printf("[F] Pot.raff  : %.0f W\n",     p_pot_raff);
    Serial.printf("[S] Pot.stampo: %+.0f W\n",    p_pot_stampo);
    Serial.printf("[Z] Setpoint  : %.1f °C\n",    p_temp_stampo_set);
    Serial.printf("Ramp stampo  : %.1f °C/s   FF gain: %.2f\n", stampo_ramp_rate, ff_gain);
    Serial.println(F("=======================================================\n"));
}

void gestisciSerial() {
    while (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() < 1) continue;

        if (input.equalsIgnoreCase("OFF")) {
            interruttore_on = false;
            Serial.println(F(">>> MACCHINA SPENTA (Interruttore generale OFF)"));
            continue;
        }
        if (input.equalsIgnoreCase("ON")) {
            interruttore_on = true;
            Serial.println(F(">>> MACCHINA ACCESA (Interruttore generale ON)"));
            continue;
        }
        if (input.equalsIgnoreCase("ACQUA")) {
            errore_acqua = true;
            digitalWrite(PIN_RELE_ACQUA, LOW); 
            in_attesa_acqua = true;
            t_attesa_acqua = millis();
            Serial.println(F(">>> Comando ACQUA: Iniezione termodinamica per 3 SECONDI..."));
            continue;
        }
        if (input.equalsIgnoreCase("PERDITA")) {
            errore_acqua = true;
            in_attesa_acqua = false; 
            digitalWrite(PIN_RELE_ACQUA, LOW); 
            Serial.println(F(">>> Comando PERDITA: Valvola rete aperta FISSA! Miscelazione continua."));
            continue;
        }
        if (input.equalsIgnoreCase("RIPRISTINA")) {
            errore_acqua = false;
            in_attesa_acqua = false; 
            digitalWrite(PIN_RELE_ACQUA, HIGH); 
            Serial.println(F(">>> RESET: Valvola rete chiusa. Macchina sbloccata!"));
            continue;
        }

        char cmd = toupper(input.charAt(0));
        if (cmd == '?') { stampaMenu(); continue; }
        if (input.length() < 2) continue;
        float val = input.substring(1).toFloat();

        switch (cmd) {
            case 'T': p_temp_rete         = val;            break;
            case 'E': p_temp_ambiente     = val;            break;
            case 'L': p_portata           = val;            break;
            case 'A': p_massa_acqua       = max(0.1f, val);  break;
            case 'M': p_massa_stampo      = max(0.1f, val);  break;
            case 'X': p_k_disp_tubi       = val;            break;
            case 'Y': p_k_disp_serbatoio  = val;            break;
            case 'R': p_pot_risc          = val;            break;
            case 'F': p_pot_raff          = val;            break;
            case 'S': p_pot_stampo        = val;            break;
            case 'Z': p_temp_stampo_set   = val;            break;
            case 'B': stampo_auto         = (val != 0.0f);   break;
            default: Serial.println(F("Comando sconosciuto")); continue;
        }
        Serial.printf(">>> [%c] = %.2f\n", cmd, val);
    }
}

// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);
    pinMode(PIN_RAFFREDDAMENTO, INPUT_PULLUP);
    pinMode(CS_MANDATA, OUTPUT); digitalWrite(CS_MANDATA, HIGH);
    pinMode(CS_RITORNO, OUTPUT); digitalWrite(CS_RITORNO, HIGH);
    
    pinMode(PIN_RELE_ACQUA, OUTPUT);
    digitalWrite(PIN_RELE_ACQUA, HIGH); 

    SPI.begin();

    resetStatoTermico();
    t_last = millis();
    stampaMenu();
}

// ══════════════════════════════════════════════════════════════════
//  LOOP — CICLO DI CONTROLLO 10 Hz
// ══════════════════════════════════════════════════════════════════
void loop() {
    gestisciSerial();

    unsigned long now = millis();

    if (in_attesa_acqua && (now - t_attesa_acqua >= 3000)) {
        in_attesa_acqua = false;
        errore_acqua = false;
        digitalWrite(PIN_RELE_ACQUA, HIGH); 
        Serial.println(F(">>> 3 SECONDI SCADUTI: Valvola rete chiusa."));
    }

    if (now - t_last < CICLO_MS) return;
    float dt = constrain((now - t_last) * 0.001f, 0.0f, DT_MAX_S);
    t_last = now;

    float portata_kgs    = max(p_portata / 60.0f, 0.001f);
    float frac_caldo_raw = leggiComandoRiscaldo();
    bool  raffreddo_on   = (digitalRead(PIN_RAFFREDDAMENTO) == LOW);

    bool macchina_spenta = !interruttore_on || errore_acqua;

    // ←←← RIMOSSO IL RESET DISTRUTTIVO ←←←
    // if (era_spenta && !macchina_spenta) {
    //     resetStatoTermico();
    // }
    era_spenta = macchina_spenta;

    float frac_caldo = 0.0f;
    float Q_ff = 0.0f;

    if (macchina_spenta) {
        frac_caldo = 0.0f;
        temp_stampo_set_attiva = p_temp_ambiente;
        portata_kgs = 0.001f; 
        raffreddo_on = false; 
    } else {
        float step_rampa = (stampo_ramp_rate / 60.0f) * dt;
        if (abs(p_temp_stampo_set - temp_stampo_set_attiva) < step_rampa) {
            temp_stampo_set_attiva = p_temp_stampo_set;
        } else {
            temp_stampo_set_attiva += (p_temp_stampo_set > temp_stampo_set_attiva) ? step_rampa : -step_rampa;
        }
        Q_ff = ff_gain * p_pot_stampo * (p_portata / 20.0f);
        frac_caldo = constrain(frac_caldo_raw + Q_ff / p_pot_risc, 0.0f, 1.0f);
    }

    float Q_in_elettrico = potenzaRiscaldo(frac_caldo, p_pot_risc);

    // ── RESISTENZE (modello a due masse) ──────────────────────
    float Q_res_to_water = K_SCAMBIO_RES * (temp_resistenza - temp_serbatoio);
    temp_resistenza += (Q_in_elettrico - Q_res_to_water) / (MASSA_RES_KG * CP_ACCIAIO) * dt;

    float Q_nucleo_to_acqua = K_NUCLEO * (temp_nucleo_res - temp_serbatoio);
    temp_nucleo_res += (-Q_nucleo_to_acqua) / (M_NUCLEO * CP_ACQUA) * dt;

    // ── RAFFREDDAMENTO PROPORZIONALE ──────────────────────────
    float Q_out = potenzaRaffreddamentoProporzionale(
        raffreddo_on, temp_serbatoio, p_temp_rete, p_pot_raff, p_massa_acqua, dt);

    // ── DISPERSIONI AMBIENTALI ────────────────────────────────
    float k_tot_disp = p_k_disp_tubi + p_k_disp_serbatoio;
    float Q_disp = k_tot_disp * (temp_serbatoio - p_temp_ambiente);

    // ── INGRESSO ACQUA DI RETE - BILANCIO DI MISCELAZIONE REALISTICO ────
    float Q_perdita_rete = 0.0f;
    if (errore_acqua) {
        // Portata acqua fredda in ingresso (circa 12 L/min)
        float portata_rete_kgs = 12.0f / 60.0f; 
        
        // Bilancio di miscelazione: acqua fredda entra e si mescola con la massa esistente
        // Consideriamo anche la massa metallica (resistenza + nucleo) per maggiore inerzia
        float massa_totale_equivalente = p_massa_acqua + (MASSA_RES_KG * 0.6f) + (M_NUCLEO * 0.4f);
        
        // Calcolo della potenza termica rimossa (acqua fredda raffredda il sistema)
        Q_perdita_rete = portata_rete_kgs * CP_ACQUA * (temp_serbatoio - p_temp_rete);
        
        // Aggiungiamo un fattore di inerzia per evitare crollo istantaneo
        Q_perdita_rete *= 0.65f;   // riduce l'effetto per simulare miscelazione graduale
    }

    // ── DEAD-TIME TRASPORTO ────────────────────────────────────
    dead_buffer[dead_buf_idx] = temp_serbatoio;
    dead_buf_idx = (dead_buf_idx + 1) % DEAD_BUF_SIZE;
    float t_mandata_serb  = dead_buffer[dead_buf_idx];
    float t_mandata_reale = t_mandata_serb 
                          - dispersioneTubi(t_mandata_serb) / (portata_kgs * CP_ACQUA);

    // ── STAMPO ────────────────────────────────────────────────
    float Q_conv = K_stampo * (t_mandata_reale - temp_stampo);
    if (macchina_spenta) Q_conv = 0.0f; 

    float carico_stampo  = macchina_spenta ? 0.0f : p_pot_stampo;
    float Q_netto_stampo = Q_conv + carico_stampo;
    
    float dT_stampo = Q_netto_stampo / (p_massa_stampo * CP_ACCIAIO) * dt;
    temp_stampo += dT_stampo;

    float dT_from_conv        = Q_conv / (portata_kgs * CP_ACQUA);
    float t_ritorno_da_stampo = t_mandata_reale - dT_from_conv;
    float t_ritorno_reale     = t_ritorno_da_stampo
                              - dispersioneTubi(t_ritorno_da_stampo) / (portata_kgs * CP_ACQUA);

    float Q_proc = portata_kgs * CP_ACQUA * (t_ritorno_reale - temp_serbatoio);

    // ── BILANCIO ENERGETICO SERBATOIO ─────────────────────────
    float pot_netta = Q_res_to_water + Q_nucleo_to_acqua + Q_proc - Q_out - Q_disp - Q_perdita_rete;
    temp_serbatoio += pot_netta / (p_massa_acqua * CP_ACQUA) * dt;

    // Protezioni fisiche
    temp_resistenza = constrain(temp_resistenza, -50.0f, 800.0f);
    temp_nucleo_res = constrain(temp_nucleo_res, -50.0f, 800.0f);
    temp_serbatoio  = constrain(temp_serbatoio,  p_temp_rete - 5.0f, 400.0f);
    temp_stampo     = constrain(temp_stampo,     p_temp_ambiente - 10.0f, 400.0f);

    // ── INERZIA SONDE PT1000 ──────────────────────────────────
    unsigned int steps   = max(1u, (unsigned int)round(CICLO_MS / (float)SENSOR_DT_MS));
    float         alpha_s = ALPHA_SONDA * (SENSOR_DT_MS * 0.001f);
    for (unsigned int i = 0; i < steps; i++) {
        temp_sonda_mandata = filtraPL(temp_sonda_mandata, t_mandata_reale, alpha_s);
        temp_sonda_ritorno = filtraPL(temp_sonda_ritorno, t_ritorno_reale, alpha_s);
    }
    temp_sonda_mandata += OFFSET_MANDATA;

    // ── SCRITTURA MCP4261 ─────────────────────────────────────
    inviaWiper(CS_MANDATA, trovaPassiPotenziometro(tempAResistenza(temp_sonda_mandata)));
    inviaWiper(CS_RITORNO, trovaPassiPotenziometro(tempAResistenza(temp_sonda_ritorno)));

    // ── LOG SERIALE ───────────────────────────────────────────
    if (now - t_log >= 2000) {
        t_log = now;
        const char* stato = errore_acqua ? (in_attesa_acqua ? "INIEZ.H2O" : "PERDITA!") : (macchina_spenta ? "SPENTA" : "ATTIVA");
        Serial.printf(
            "STATO:%s | Res:%.1f Serb:%.1f | Mand:%.1f Rit:%.1f | Stmp:%.1f | PWM:%.1f%% FF:%.0fW\n",
            stato, temp_resistenza, temp_serbatoio,
            temp_sonda_mandata, temp_sonda_ritorno,
            temp_stampo, frac_caldo * 100.0f, Q_ff
        );
    }
}