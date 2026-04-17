# EFPq
#include <Arduino.h>
#include <SPI.h>
#include "system.h"
#include "math_handling.h"
#include "table.h"

// ══════════════════════════════════════════════════════════════════
//  PARAMETRI E STATO (Definizioni Globali)
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

float temp_resistenza    = 20.0f;
float temp_nucleo_res    = 20.0f;
float temp_serbatoio     = 20.0f;
float temp_sonda_mandata = 20.0f;
float temp_sonda_ritorno = 20.0f;
float temp_stampo        = 20.0f;
float adc_filtrato       = 0.0f;

float dead_buffer[DEAD_BUF_SIZE];
int   dead_buf_idx = 0;

bool          errore_acqua          = false;
bool          in_attesa_acqua       = false;
unsigned long t_attesa_acqua        = 0;
bool          raffreddamento_cmd    = false;
bool          raffreddo_cmd_pendente = false;
unsigned long t_raff_cmd            = 0;
bool          perdita_attiva        = false;

unsigned long t_last = 0;
unsigned long t_log  = 0;

// ══════════════════════════════════════════════════════════════════
//  FUNZIONI DI SUPPORTO
// ══════════════════════════════════════════════════════════════════

void resetStatoTermico() {
    float t0 = p_temp_rete;
    temp_resistenza = temp_nucleo_res = temp_serbatoio = temp_sonda_mandata = temp_sonda_ritorno = t0;
    temp_stampo = p_temp_ambiente; // Inizializza correttamente a T ambiente
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
    return (adc_filtrato < DEADBAND_CALDO) ? 0.0f : adc_filtrato;
}

void stampaMenu() {
    Serial.println(F("\n========= TERMOREGOLATORE DIGITAL TWIN ========="));
    Serial.println(F("Comandi: T E L A M X Y R F S V ? | ACQUA / PERDITA / RAFFREDDAMENTO / RIPRISTINA"));
    Serial.printf("[T] Rete: %.1f | [E] Amb: %.1f | [L] Portata: %.1f | [A] Massa H2O: %.1f\n", p_temp_rete, p_temp_ambiente, p_portata, p_massa_acqua);
    Serial.printf("[M] Massa Stmp: %.1f | [X] Disp Tubi: %.2f | [Y] Disp Serb: %.2f\n", p_massa_stampo, p_k_disp_tubi, p_k_disp_serbatoio);
    Serial.printf("[R] Pot Risc: %.0f | [F] Pot Raff: %.0f | [S] Pot Stmp: %+.0f | [V] Valvola: %.1f\n", p_pot_risc, p_pot_raff, p_pot_stampo, p_portata_valvola);
    Serial.println(F("================================================\n"));
}

void gestisciSerial() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("ACQUA")) {
        errore_acqua = true; in_attesa_acqua = true; t_attesa_acqua = millis();
        digitalWrite(PIN_RELE_ACQUA, LOW); return;
    }
    if (input.equalsIgnoreCase("PERDITA")) {
        errore_acqua = true; perdita_attiva = true;
        digitalWrite(PIN_RELE_ACQUA, LOW); return;
    }
    if (input.equalsIgnoreCase("RAFFREDDAMENTO")) {
        raffreddo_cmd_pendente = true; t_raff_cmd = millis(); return;
    }
    if (input.equalsIgnoreCase("RIPRISTINA")) {
        errore_acqua = false; perdita_attiva = false; raffreddamento_cmd = false;
        digitalWrite(PIN_RELE_ACQUA, HIGH); return;
    }
    
    char cmd = toupper(input.charAt(0));
    if (cmd == '?') { stampaMenu(); return; }
    
    float val = input.substring(1).toFloat();
    switch (cmd) {
        case 'T': p_temp_rete = val; break;
        case 'E': p_temp_ambiente = val; break;
        case 'L': p_portata = val; break;
        case 'A': p_massa_acqua = max(0.1f, val); break;
        case 'M': p_massa_stampo = max(0.1f, val); break;
        case 'X': p_k_disp_tubi = val; break;
        case 'Y': p_k_disp_serbatoio = val; break;
        case 'R': p_pot_risc = val; break;
        case 'F': p_pot_raff = val; break;
        case 'S': p_pot_stampo = val; break;
        case 'V': p_portata_valvola = val; break;
    }
}

// ══════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ══════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);
    pinMode(PIN_RAFFREDDAMENTO, INPUT_PULLUP);
    pinMode(CS_MANDATA, OUTPUT); digitalWrite(CS_MANDATA, HIGH);
    pinMode(CS_RITORNO, OUTPUT); digitalWrite(CS_RITORNO, HIGH);
    pinMode(PIN_RELE_ACQUA, OUTPUT); digitalWrite(PIN_RELE_ACQUA, HIGH);
    SPI.begin();
    resetStatoTermico();
    t_last = millis();
    stampaMenu();
}

void loop() {
    gestisciSerial();

    unsigned long now = millis();
    if (in_attesa_acqua && (now - t_attesa_acqua >= 3000UL)) {
        in_attesa_acqua = false; errore_acqua = false;
        digitalWrite(PIN_RELE_ACQUA, HIGH);
    }

    if (now - t_last < CICLO_MS) return;
    float dt = constrain((now - t_last) * 0.001f, 0.0f, DT_MAX_S);
    t_last = now;

    float portata_kgs = errore_acqua ? 0.001f : max(p_portata / 60.0f, 0.001f);
    float frac_caldo_raw = leggiComandoRiscaldo();

    if (raffreddo_cmd_pendente) {
        if (now - t_raff_cmd > TIMEOUT_RAFF_CONF) {
            raffreddo_cmd_pendente = false;
        } else if (digitalRead(PIN_RAFFREDDAMENTO) == LOW) {
            raffreddo_cmd_pendente = false;
            raffreddamento_cmd = true;
        }
    }

    bool raffreddo_on = raffreddamento_cmd;
    bool macchina_spenta = (frac_caldo_raw <= 0.0f && !raffreddo_on) || errore_acqua;

    float Q_ff = -ff_gain * p_pot_stampo * (p_portata / 20.0f);
    float frac_caldo = macchina_spenta ? 0.0f : constrain(frac_caldo_raw + Q_ff / p_pot_risc, 0.0f, 1.0f);
    float Q_in_elettrico = potenzaRiscaldo(frac_caldo, p_pot_risc);

    // ── MODELLO HEATER ─────────────────────────────────────────────
    float Q_res_to_nucleo = K_SCAMBIO_RES * (temp_resistenza - temp_nucleo_res);
    temp_resistenza += (Q_in_elettrico - Q_res_to_nucleo) / (MASSA_RES_KG * CP_ACCIAIO) * dt;

    float Q_nucleo_to_acqua = K_NUCLEO * (temp_nucleo_res - temp_serbatoio);
    temp_nucleo_res += (Q_res_to_nucleo - Q_nucleo_to_acqua) / (M_NUCLEO * CP_ACCIAIO) * dt;

    // ── RAFFREDDAMENTO ──────────────────────────────────────────────
    float portata_v_kgs = (raffreddo_on || (errore_acqua && in_attesa_acqua)) ? p_portata_valvola/60.0f : (perdita_attiva ? PORTATA_PERDITA_LMIN/60.0f : 0.0f);
    float Q_out = min(potenzaRaffreddamento((portata_v_kgs > 0), temp_serbatoio, p_temp_rete, portata_v_kgs), p_pot_raff);

    // ── MANDATA E DEAD-TIME ────────────────────────────────────────
    dead_buffer[dead_buf_idx] = temp_serbatoio;
    dead_buf_idx = (dead_buf_idx + 1) % DEAD_BUF_SIZE;
    float t_mandata_reale = dead_buffer[dead_buf_idx] - (dispersioneTubi(dead_buffer[dead_buf_idx]) / (portata_kgs * CP_ACQUA));

    // ── LOGICA STAMPO (CORRETTA) ───────────────────────────────────
    // Bilancio: Accumulo = Convezione Fluido + Carico Interno - Dispersione Ambiente
    float Q_conv = K_stampo * (t_mandata_reale - temp_stampo);
    float Q_disp_stampo = 4.0f * (temp_stampo - p_temp_ambiente); // Coeff. dispersione naturale
    float Q_netto_stampo = Q_conv + (macchina_spenta ? 0.0f : p_pot_stampo) - Q_disp_stampo;
    
    temp_stampo += (Q_netto_stampo / (p_massa_stampo * CP_ACCIAIO)) * dt;

    // ── RITORNO E SERBATOIO ────────────────────────────────────────
    float t_ritorno_reale = t_mandata_reale - (Q_conv / (portata_kgs * CP_ACQUA));
    float Q_proc = portata_kgs * CP_ACQUA * (t_ritorno_reale - temp_serbatoio);
    
    float pot_netta = Q_nucleo_to_acqua + Q_proc - Q_out - (p_k_disp_serbatoio * (temp_serbatoio - p_temp_ambiente));
    temp_serbatoio += (pot_netta / (p_massa_acqua * CP_ACQUA)) * dt;

    // ── FILTRO SONDE E USCITA ──────────────────────────────────────
    float alpha_s = ALPHA_SONDA * (SENSOR_DT_MS * 0.001f);
    temp_sonda_mandata = filtraPL(temp_sonda_mandata, t_mandata_reale, alpha_s);
    temp_sonda_ritorno = filtraPL(temp_sonda_ritorno, t_ritorno_reale, alpha_s);

    inviaWiper(CS_MANDATA, trovaPassiPotenziometro(tempAResistenza(temp_sonda_mandata + OFFSET_MANDATA)));
    inviaWiper(CS_RITORNO, trovaPassiPotenziometro(tempAResistenza(temp_sonda_ritorno + OFFSET_RITORNO)));

    // ── LOG SERIALE (PWM RIPRISTINATO) ──────────────────────────────
    if (now - t_log >= 2000UL) {
        t_log = now;
        Serial.printf("STATO:%-8s | PWM:%5.1f%% | Serb:%5.1f | Mand:%5.1f Rit:%5.1f | Stmp:%5.1f | Qout:%5.0fW\n",
            macchina_spenta ? "OFF" : "ON", 
            frac_caldo * 100.0f, 
            temp_serbatoio, 
            temp_sonda_mandata + OFFSET_MANDATA, 
            temp_sonda_ritorno + OFFSET_RITORNO, 
            temp_stampo, 
            Q_out);
    }
}