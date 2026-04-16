// ══════════════════════════════════════════════════════════════════
//  EFP.ino  —  Digital Twin Termoregolatore
// ══════════════════════════════════════════════════════════════════
#include <Arduino.h>
#include <SPI.h>
#include "system.h"
#include "math_handling.h"
#include "table.h"

// ── PIN RELÈ ACQUA DI RETE ────────────────────────────────────────
#define PIN_RELE_ACQUA 27

// ══════════════════════════════════════════════════════════════════
//  PARAMETRI CONFIGURABILI
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
float ff_gain            = 1.0f;

float p_portata_valvola  = 8.0f;

// ══════════════════════════════════════════════════════════════════
//  STATO TERMODINAMICO
// ══════════════════════════════════════════════════════════════════
float temp_resistenza    = 15.0f;
float temp_nucleo_res    = 15.0f;
float temp_serbatoio     = 15.0f;
float temp_sonda_mandata = 15.0f;
float temp_sonda_ritorno = 15.0f;
float temp_stampo        = 20.0f;
float adc_filtrato       = 0.0f;

// ── FLAG DI CONTROLLO ─────────────────────────────────────────────
bool errore_acqua       = false;
bool in_attesa_acqua    = false;
unsigned long t_attesa_acqua = 0;

bool raffreddamento_cmd = false;

bool raffreddo_cmd_pendente = false;
unsigned long t_raff_cmd    = 0;
const unsigned long TIMEOUT_RAFF_CONF = 5000UL;

bool perdita_attiva     = false;

// Buffer Dead-time
static const int DEAD_BUF_SIZE = (int)(DEADTIME_S / (CICLO_MS * 0.001f) + 0.5f);
float dead_buffer[DEAD_BUF_SIZE];
int   dead_buf_idx = 0;

unsigned long t_last = 0;
unsigned long t_log  = 0;

// ══════════════════════════════════════════════════════════════════
//  FUNZIONI DI SUPPORTO
// ══════════════════════════════════════════════════════════════════
void resetStatoTermico() {
    float t0 = p_temp_rete;
    temp_resistenza  = temp_nucleo_res = t0;
    temp_serbatoio   = temp_sonda_mandata = temp_sonda_ritorno = t0;
    temp_stampo      = p_temp_ambiente;
    for (int i = 0; i < DEAD_BUF_SIZE; i++) dead_buffer[i] = t0;
    dead_buf_idx = 0;
    adc_filtrato = 0.0f;
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
    if (raw >= 2100) {
        adc_filtrato = filtraPL(adc_filtrato, 1.0f, ADC_ALPHA * 1.8f);
        return 1.0f;
    }

    float frac = constrain((float)raw / ADC_CALDO_MAX, 0.0f, 1.0f);
    adc_filtrato = filtraPL(adc_filtrato, frac, ADC_ALPHA);
    if (isnan(adc_filtrato) || isinf(adc_filtrato)) adc_filtrato = 0.0f;

    return (adc_filtrato < DEADBAND_CALDO) ? 0.0f : adc_filtrato;
}

// ══════════════════════════════════════════════════════════════════
//  MENU SERIALE
// ══════════════════════════════════════════════════════════════════
void stampaMenu() {
    float tau_v   = (p_portata_valvola > 0.01f) ? (p_massa_acqua / (p_portata_valvola / 60.0f)) : 9999.0f;

    Serial.println(F("\n========= TERMOREGOLATORE DIGITAL TWIN ========="));
    Serial.println(F("Comandi: T E L A M X Y R F S V ? | ACQUA / PERDITA / RAFFREDDAMENTO / RIPRISTINA"));
    Serial.println(F("-------------------------------------------------------"));
    Serial.printf("[T] Rete          : %.1f °C\n",    p_temp_rete);
    Serial.printf("[E] Ambiente       : %.1f °C\n",    p_temp_ambiente);
    Serial.printf("[L] Portata stampo : %.1f L/min\n", p_portata);
    Serial.printf("[A] Massa H2O      : %.1f kg\n",    p_massa_acqua);
    Serial.printf("[M] Massa stampo   : %.1f kg\n",    p_massa_stampo);
    Serial.printf("[X] Disp. tubi     : %.2f W/°C\n",  p_k_disp_tubi);
    Serial.printf("[Y] Disp. serbatoio: %.2f W/°C\n",  p_k_disp_serbatoio);
    Serial.printf("[R] Pot. riscaldo  : %.0f W\n",     p_pot_risc);
    Serial.printf("[F] Pot. raff. max : %.0f W\n",     p_pot_raff);
    Serial.printf("[S] Pot. stampo    : %+.0f W\n",    p_pot_stampo);
    Serial.println(F("-------------------------------------------------------"));
    Serial.printf("[V] Portata valvola: %.1f L/min   tau ~ %.0f s\n", p_portata_valvola, tau_v);
    Serial.println(F("=======================================================\n"));
}

void gestisciSerial() {
    while (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() < 1) continue;

        if (input.equalsIgnoreCase("ACQUA")) {
            errore_acqua    = true;
            in_attesa_acqua = true;
            t_attesa_acqua  = millis();
            digitalWrite(PIN_RELE_ACQUA, LOW);
            float tau = (p_portata_valvola > 0.01f) ? (p_massa_acqua / (p_portata_valvola / 60.0f)) : 9999.0f;
            Serial.printf(">>> ACQUA: valvola aperta %.1f L/min per 3 s  (tau ~ %.0f s)\n", p_portata_valvola, tau);
            continue;
        }

        if (input.equalsIgnoreCase("PERDITA")) {
            errore_acqua    = true;
            in_attesa_acqua = false;
            perdita_attiva  = true;
            digitalWrite(PIN_RELE_ACQUA, LOW);
            float tau = p_massa_acqua / (p_portata_valvola / 60.0f);
            Serial.printf(">>> PERDITA ATTIVA: perdita da tubo → valvola aperta a %.1f L/min  (tau ~ %.0f s)\n", 
                          p_portata_valvola, tau);
            continue;
        }

        if (input.equalsIgnoreCase("RAFFREDDAMENTO")) {
            raffreddo_cmd_pendente = true;
            t_raff_cmd = millis();
            Serial.println(F(">>> RAFFREDDAMENTO: attesa conferma PIN 4 LOW entro 5 secondi..."));
            continue;
        }

        if (input.equalsIgnoreCase("RIPRISTINA")) {
            errore_acqua           = false;
            in_attesa_acqua        = false;
            perdita_attiva         = false;
            raffreddamento_cmd     = false;
            raffreddo_cmd_pendente = false;
            digitalWrite(PIN_RELE_ACQUA, HIGH);
            Serial.println(F(">>> RIPRISTINA: valvola chiusa, sistema sbloccato."));
            continue;
        }

        char cmd = toupper(input.charAt(0));
        if (cmd == '?') { stampaMenu(); continue; }
        if (input.length() < 2) continue;
        float val = input.substring(1).toFloat();

        switch (cmd) {
            case 'T': p_temp_rete         = val; break;
            case 'E': p_temp_ambiente     = val; break;
            case 'L': p_portata           = val; break;
            case 'A': p_massa_acqua       = max(0.1f, val); break;
            case 'M': p_massa_stampo      = max(0.1f, val); break;
            case 'X': p_k_disp_tubi       = val; break;
            case 'Y': p_k_disp_serbatoio  = val; break;
            case 'R': p_pot_risc          = max(0.0f, val); break;
            case 'F': p_pot_raff          = val; break;
            case 'S': p_pot_stampo        = val; break;
            case 'V': {
                p_portata_valvola = constrain(val, 0.1f, 50.0f);
                float tau = p_massa_acqua / (p_portata_valvola / 60.0f);
                Serial.printf(">>> [V] Portata valvola = %.1f L/min  (tau ~ %.0f s)\n", p_portata_valvola, tau);
                continue;
            }
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
//  LOOP
// ══════════════════════════════════════════════════════════════════
void loop() {
    gestisciSerial();

    unsigned long now = millis();

    if (in_attesa_acqua && (now - t_attesa_acqua >= 3000UL)) {
        in_attesa_acqua = false;
        errore_acqua    = false;
        digitalWrite(PIN_RELE_ACQUA, HIGH);
        Serial.println(F(">>> 3 s scaduti: valvola chiusa, macchina sbloccata."));
    }

    if (now - t_last < CICLO_MS) return;
    float dt = constrain((now - t_last) * 0.001f, 0.0f, DT_MAX_S);
    t_last = now;

    float portata_kgs    = max(p_portata / 60.0f, 0.001f);
    float frac_caldo_raw = leggiComandoRiscaldo();
    if (isnan(frac_caldo_raw) || isinf(frac_caldo_raw)) frac_caldo_raw = 0.0f;

    bool raffreddo_on = false;

    if (raffreddo_cmd_pendente) {
        if (millis() - t_raff_cmd > TIMEOUT_RAFF_CONF) {
            raffreddo_cmd_pendente = false;
            Serial.println(F(">>> Timeout 5s: conferma PIN non arrivata → RAFFREDDAMENTO annullato"));
        } 
        else if (digitalRead(PIN_RAFFREDDAMENTO) == LOW) {
            raffreddo_cmd_pendente = false;
            raffreddamento_cmd = true;
            Serial.println(F(">>> Conferma PIN 4 LOW ricevuta → RAFFREDDAMENTO ATTIVATO"));
        }
    }

    raffreddo_on = raffreddamento_cmd;

    bool macchina_spenta = (frac_caldo_raw <= 0.0f && !raffreddo_on) || errore_acqua;

    float frac_caldo = 0.0f;
    float Q_ff       = 0.0f;

    if (macchina_spenta) {
        frac_caldo   = 0.0f;
        portata_kgs  = 0.001f;
    } else {
        Q_ff       = ff_gain * p_pot_stampo * (p_portata / 20.0f);
        frac_caldo = constrain(frac_caldo_raw + Q_ff / p_pot_risc, 0.0f, 1.0f);
        if (isnan(frac_caldo) || isinf(frac_caldo)) frac_caldo = 0.0f;
    }

    float Q_in_elettrico = potenzaRiscaldo(frac_caldo, p_pot_risc);

    float Q_res_to_water    = K_SCAMBIO_RES * (temp_resistenza - temp_serbatoio);
    temp_resistenza        += (Q_in_elettrico - Q_res_to_water) / (MASSA_RES_KG * CP_ACCIAIO) * dt;

    float Q_nucleo_to_acqua = K_NUCLEO * (temp_nucleo_res - temp_serbatoio);
    temp_nucleo_res        += (-Q_nucleo_to_acqua) / (M_NUCLEO * CP_ACQUA) * dt;

    float portata_valvola_kgs = 0.0f;
    if (raffreddo_on || (errore_acqua && (in_attesa_acqua || perdita_attiva))) {
        portata_valvola_kgs = p_portata_valvola / 60.0f;
    }

    float Q_out = potenzaRaffreddamento((portata_valvola_kgs > 0.0f), temp_serbatoio, p_temp_rete, portata_valvola_kgs);

    // ── DISPERSIONI AMBIENTALI ────────────────────────────────────
    // MODIFICA QUI: logica termodinamica corretta e più efficace
    // Il serbatoio scambia calore con l'ambiente INDEPENDENTEMENTE dalla portata dello stampo
    // Q_disp > 0  → perde calore (si raffredda se T_amb < T_serb)
    // Q_disp < 0  → guadagna calore (si scalda se T_amb > T_serb)
    float k_disp_ambiente = p_k_disp_tubi + p_k_disp_serbatoio;
    float Q_disp = k_disp_ambiente * (temp_serbatoio - p_temp_ambiente);

    // ── DEAD-TIME + MANDATA / RITORNO ────────────────────────────
    dead_buffer[dead_buf_idx] = temp_serbatoio;
    dead_buf_idx = (dead_buf_idx + 1) % DEAD_BUF_SIZE;
    float t_mandata_serb = dead_buffer[dead_buf_idx];

    float t_mandata_reale = t_mandata_serb;
    if (portata_kgs > 0.002f)
        t_mandata_reale -= dispersioneTubi(t_mandata_serb) / (portata_kgs * CP_ACQUA);

    float Q_disp_stampo = 3.0f * (temp_stampo - p_temp_ambiente);
    float Q_conv = macchina_spenta ? 0.0f : K_stampo * (t_mandata_reale - temp_stampo);
    float Q_netto_stampo = Q_conv + (macchina_spenta ? 0.0f : p_pot_stampo) - Q_disp_stampo;
    temp_stampo += Q_netto_stampo / (p_massa_stampo * CP_ACCIAIO) * dt;

    float t_ritorno_reale = t_mandata_reale;
    if (portata_kgs > 0.002f) {
        float dT_from_conv        = Q_conv / (portata_kgs * CP_ACQUA);
        float t_ritorno_da_stampo = t_mandata_reale - dT_from_conv;
        t_ritorno_reale = t_ritorno_da_stampo 
                        - dispersioneTubi(t_ritorno_da_stampo) / (portata_kgs * CP_ACQUA);
    }

    float Q_proc = (portata_kgs > 0.002f) 
                   ? portata_kgs * CP_ACQUA * (t_ritorno_reale - temp_serbatoio) 
                   : 0.0f;

    // ── BILANCIO SERBATOIO ────────────────────────────────────────
    float pot_netta = Q_res_to_water + Q_nucleo_to_acqua + Q_proc - Q_out - Q_disp;
    temp_serbatoio += pot_netta / (p_massa_acqua * CP_ACQUA) * dt;

    // Protezioni
    if (isnan(temp_serbatoio)  || isinf(temp_serbatoio))  temp_serbatoio  = p_temp_rete;
    if (isnan(temp_resistenza) || isinf(temp_resistenza)) temp_resistenza = p_temp_rete;
    if (isnan(temp_nucleo_res) || isinf(temp_nucleo_res)) temp_nucleo_res = p_temp_rete;
    if (isnan(temp_stampo)     || isinf(temp_stampo))     temp_stampo     = p_temp_ambiente;

    temp_serbatoio  = constrain(temp_serbatoio,  p_temp_rete,          400.0f);
    temp_resistenza = constrain(temp_resistenza, -50.0f,               800.0f);
    temp_nucleo_res = constrain(temp_nucleo_res, -50.0f,               800.0f);
    temp_stampo     = constrain(temp_stampo,      p_temp_ambiente - 10.0f, 400.0f);

    // Inerzia sonde
    unsigned int steps   = max(1u, (unsigned int)round(CICLO_MS / (float)SENSOR_DT_MS));
    float        alpha_s = ALPHA_SONDA * (SENSOR_DT_MS * 0.001f);
    for (unsigned int i = 0; i < steps; i++) {
        temp_sonda_mandata = filtraPL(temp_sonda_mandata, t_mandata_reale, alpha_s);
        temp_sonda_ritorno = filtraPL(temp_sonda_ritorno, t_ritorno_reale, alpha_s);
    }
    temp_sonda_mandata += OFFSET_MANDATA;

    // Scrittura MCP4261
    inviaWiper(CS_MANDATA, trovaPassiPotenziometro(tempAResistenza(temp_sonda_mandata)));
    inviaWiper(CS_RITORNO, trovaPassiPotenziometro(tempAResistenza(temp_sonda_ritorno)));

    // Log
    if (now - t_log >= 2000UL) {
        t_log = now;

        const char* stato;
        if      (errore_acqua && in_attesa_acqua)  stato = "INIEZ.H2O";
        else if (errore_acqua && perdita_attiva)   stato = "PERDITA!";
        else if (errore_acqua)                     stato = "ERRORE";
        else if (raffreddamento_cmd)               stato = "RAFF.CMD";
        else if (raffreddo_on)                     stato = "RAFFREDD.";
        else if (macchina_spenta)                  stato = "SPENTA";
        else                                       stato = "ATTIVA";

        Serial.printf(
            "STATO:%-9s | Res:%5.1f Serb:%5.1f | Mand:%5.1f Rit:%5.1f"
            " | Stmp:%5.1f | PWM:%5.1f%% | Qout:%6.0fW [V=%.1f L/m]\n",
            stato, temp_resistenza, temp_serbatoio,
            temp_sonda_mandata, temp_sonda_ritorno, temp_stampo,
            (isnan(frac_caldo) ? 0.0f : frac_caldo * 100.0f),
            Q_out, p_portata_valvola
        );
    }
}
