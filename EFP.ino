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
float p_k_hx             = 800.0f;
// [W/°C] Coefficiente scambio HX 
float p_pot_stampo       = 0.0f;
float p_temp_ambiente    = 20.0f;
float p_k_disp_tubi      = 2.5f;
float p_k_disp_serbatoio = 5.0f;
float p_massa_stampo     = 15.0f;
float K_stampo           = K_STAMPO_DEFAULT;
float ff_gain            = 1.0f;
float p_portata_valvola  = 8.0f;
// [L/min] Solo iniezione diretta

// --- NUOVI PARAMETRI DIGITAL TWIN AVANZATO ---
bool  stato_pompa        = true;
// Pompa di circolazione
float p_pot_pompa        = 200.0f;
// [W] Calore meccanico immesso dalla pompa
float apertura_valvola_hx= 0.0f;     // [0.0 - 1.0] Inerzia fisica della valvola freddo
const float CP_OLIO_VAL  = 2100.0f;
float cp_fluido          = CP_ACQUA;
// Variabile dipendente dal fluido (default 4186)
float t_ebollizione      = 100.0f;
// Limite termodinamico
bool  is_olio            = false;
// ---------------------------------------------

float temp_resistenza    = 20.0f;
float temp_nucleo_res    = 20.0f;
float temp_serbatoio     = 20.0f;
float temp_sonda_mandata = 20.0f;
float temp_sonda_ritorno = 20.0f;
float temp_stampo        = 20.0f;
float adc_filtrato       = 0.0f;

float dead_buffer[DEAD_BUF_SIZE];
int   dead_buf_idx = 0;
bool          errore_acqua           = false;
bool          in_attesa_acqua        = false;
unsigned long t_attesa_acqua         = 0;
bool          raffreddamento_cmd     = false;
bool          raffreddo_cmd_pendente = false;
unsigned long t_raff_cmd             = 0;
bool          perdita_attiva         = false;
bool          raffreddamento_attivo  = false;
unsigned long t_last = 0;
unsigned long t_log  = 0;
// ══════════════════════════════════════════════════════════════════
//  FUNZIONI DI SUPPORTO
// ══════════════════════════════════════════════════════════════════

void resetStatoTermico() {
    float t0 = p_temp_ambiente;
// Usa l'ambiente come base, non la rete
    temp_resistenza = temp_nucleo_res = temp_serbatoio =
        temp_sonda_mandata = temp_sonda_ritorno = t0;
temp_stampo = t0;
    for (int i = 0; i < DEAD_BUF_SIZE; i++) dead_buffer[i] = t0;
    dead_buf_idx = 0;
adc_filtrato = 0.0f;
}
    

void inviaWiper(uint8_t cs, bytes8_t passi) {
    uint8_t cmd0 = 0x00 |
(passi.p1 > 255 ? 0x01 : 0x00);
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
Serial.println(F("Comandi: T E L A M X Y R F S V ? | P1/P0 (Pompa) | OLIO/H2O"));
Serial.println(F("Avanzati: ACQUA / PERDITA / RAFFREDDAMENTO / RIPRISTINA / STOP"));
Serial.printf("[T] Rete: %.1f°C | [E] Amb: %.1f°C | [L] Portata: %.1f L/min | [A] Massa fluido: %.1f kg\n",
        p_temp_rete, p_temp_ambiente, p_portata, p_massa_acqua);
Serial.printf("[M] Massa Stmp: %.1f kg | [X] Disp Tubi: %.2f W/°C | [Y] Disp Serb: %.2f W/°C\n",
        p_massa_stampo, p_k_disp_tubi, p_k_disp_serbatoio);
Serial.printf("[R] Pot Risc: %.0f W | [F] HX k: %.0f W/°C | [S] Pot Stmp: %+.0f W | [V] Valvola: %.1f L/min\n",
        p_pot_risc, p_k_hx, p_pot_stampo, p_portata_valvola);
Serial.printf("[P] Pompa: %s (%.0fW) | [O] Fluido: %s (Ebollizione a %.0f°C)\n",
        stato_pompa ? "ON" : "OFF", p_pot_pompa, is_olio ? "OLIO" : "ACQUA", t_ebollizione);
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
        errore_acqua = true;
perdita_attiva = true;
        digitalWrite(PIN_RELE_ACQUA, LOW); return;
    }
    if (input.equalsIgnoreCase("RAFFREDDAMENTO")) {
        raffreddamento_attivo = true;
        return;
}
    if (input.equalsIgnoreCase("STOP")) {
        raffreddamento_attivo = false;
        digitalWrite(PIN_RELE_ACQUA, HIGH);
        return;
}
    if (input.equalsIgnoreCase("RIPRISTINA")) {
        errore_acqua = false;
        perdita_attiva = false;
raffreddamento_cmd = false;
        raffreddamento_attivo = false;
        digitalWrite(PIN_RELE_ACQUA, HIGH); return;
    }

    // Nuovi comandi Digital Twin
    if (input.equalsIgnoreCase("P1")) { stato_pompa = true;
Serial.println("Pompa ON"); return; }
    if (input.equalsIgnoreCase("P0")) { stato_pompa = false; Serial.println("Pompa OFF"); return;
}
    if (input.equalsIgnoreCase("OLIO")) { is_olio = true; cp_fluido = CP_OLIO_VAL; t_ebollizione = 300.0f; Serial.println("Fluido: OLIO"); return;
}
    if (input.equalsIgnoreCase("H2O")) { is_olio = false; cp_fluido = CP_ACQUA; t_ebollizione = 100.0f; Serial.println("Fluido: ACQUA"); return;
}

    char cmd = toupper(input.charAt(0));
    if (cmd == '?') { stampaMenu(); return;
}

    float val = input.substring(1).toFloat();
    switch (cmd) {
        case 'T': p_temp_rete        = val;
break;
        case 'E': p_temp_ambiente    = val;              break;
case 'L': p_portata          = val;              break;
case 'A': p_massa_acqua      = max(0.1f, val);   break;
case 'M': p_massa_stampo     = max(0.1f, val);   break;
case 'X': p_k_disp_tubi      = val;              break;
        case 'Y': p_k_disp_serbatoio = val;              break;
case 'R': p_pot_risc         = max(1.0f, val);   break; 
case 'F': p_k_hx             = max(0.0f, val);   break;
case 'S': p_pot_stampo       = val;              break;
        case 'V': p_portata_valvola  = val;              break;
}
}

// ══════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ══════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);
    pinMode(PIN_RAFFREDDAMENTO, INPUT_PULLUP);
pinMode(CS_MANDATA,    OUTPUT); digitalWrite(CS_MANDATA,    HIGH);
    pinMode(CS_RITORNO,    OUTPUT); digitalWrite(CS_RITORNO,    HIGH);
pinMode(PIN_RELE_ACQUA, OUTPUT); digitalWrite(PIN_RELE_ACQUA, HIGH);
    SPI.begin();
    resetStatoTermico();
    t_last = millis();
    stampaMenu();
}

void loop() {
    gestisciSerial();
unsigned long now = millis();

    // Lo scambiatore segue direttamente lo stato del Pin 4
    if (digitalRead(PIN_RAFFREDDAMENTO) == LOW) {
        raffreddamento_cmd = true;
} else {
        raffreddamento_cmd = false;
}

    // ── Auto-reset errore acqua dopo 3 s ──────────────────────────
    if (in_attesa_acqua && (now - t_attesa_acqua >= 3000UL)) {
        in_attesa_acqua = false;
errore_acqua = false;
        digitalWrite(PIN_RELE_ACQUA, HIGH);
    }

    if (now - t_last < CICLO_MS) return;
float dt = constrain((now - t_last) * 0.001f, 0.0f, DT_MAX_S);
    t_last = now;
// ── Portata circolazione (Sganciata dal PID) ──────────────────
    // La circolazione e il processo non si fermano con il PID a 0%
    float portata_kgs = stato_pompa ?
max(p_portata / 60.0f, 0.001f) : 0.001f;
    bool circolazione_attiva = stato_pompa;
// ── Lettura comando riscaldamento ─────────────────────────────
    float frac_caldo_raw = leggiComandoRiscaldo();
float Q_ff       = -ff_gain * p_pot_stampo * (p_portata / 20.0f);
float frac_caldo = (!circolazione_attiva) 
                       ?
0.0f 
                       : constrain(frac_caldo_raw + Q_ff / p_pot_risc, 0.0f, 1.0f);
float Q_in_elettrico = potenzaRiscaldo(frac_caldo, p_pot_risc);

    // ── MODELLO HEATER ─────────────────────────────────────────────
    float Q_res_to_nucleo  = K_SCAMBIO_RES * (temp_resistenza  - temp_nucleo_res);
    {
        float delta_T_res = (Q_in_elettrico - Q_res_to_nucleo) / (MASSA_RES_KG * CP_ACCIAIO) * dt;
        if (delta_T_res >  50.0f) delta_T_res =  50.0f;
        if (delta_T_res < -50.0f) delta_T_res = -50.0f;
        temp_resistenza += delta_T_res;
    }

    float Q_nucleo_to_acqua = K_NUCLEO * (temp_nucleo_res - temp_serbatoio);
    temp_nucleo_res += (Q_res_to_nucleo - Q_nucleo_to_acqua) / (M_NUCLEO * CP_ACCIAIO) * dt;
// ── RAFFREDDAMENTO ─────────────────────────────────────────────
    
    // 1) Inerzia Attuatore Valvola (Tempo di corsa ~3s)
    float target_apertura = (raffreddamento_cmd || raffreddamento_attivo) ?
1.0f : 0.0f;
    float alpha_valvola   = constrain(dt / 3.0f, 0.0f, 1.0f);
// Filtro con costante tempo di 3s
    apertura_valvola_hx   = filtraPL(apertura_valvola_hx, target_apertura, alpha_valvola);
// 2) Coefficienti Dinamici di Scambio Termico (Reynolds)
    // Se la portata scende, il moto laminare abbassa il K di scambio (20 L/min è la nominale)
    float rapporto_portata   = max(p_portata / 20.0f, 0.01f);
float K_hx_effettivo     = p_k_hx * pow(rapporto_portata, 0.8f);
    float K_stampo_effettivo = K_stampo * pow(rapporto_portata, 0.8f);
float Q_scambiatore_nominale = potenzaScambiatore(
        true, // Calcolato sempre, limitato poi meccanicamente dall'apertura
        temp_serbatoio,
        p_temp_rete,
        K_hx_effettivo
    );
// Potenza reale modulata dall'inerzia della valvola
    float Q_scambiatore = Q_scambiatore_nominale * apertura_valvola_hx;
// 3) Iniezione diretta
    float portata_iniet_kgs = 0.0f;
if (errore_acqua && in_attesa_acqua) {
        portata_iniet_kgs = p_portata_valvola / 60.0f;
} else if (perdita_attiva) {
        portata_iniet_kgs = PORTATA_PERDITA_LMIN / 60.0f;
}

    float Q_iniezione = 0.0f;
    if (portata_iniet_kgs > 0.0f) {
        Q_iniezione = portata_iniet_kgs * cp_fluido * (temp_serbatoio - p_temp_rete);
        p_massa_acqua += portata_iniet_kgs * dt;
        if (p_massa_acqua > 20.0f) p_massa_acqua = 20.0f;
}
    float Q_out = Q_scambiatore + Q_iniezione;
// ── MANDATA E DEAD-TIME DINAMICO ───────────────────────────────
    dead_buffer[dead_buf_idx] = temp_serbatoio;
// Il ritardo dipende dal volume tubi (es. stimato a 2L) e dalla portata
    float volume_tubi_L = 2.0f;
float tempo_percorrenza_s = (p_portata > 0.1f) ? (volume_tubi_L / (p_portata / 60.0f)) : 9999.0f;
    int offset_cicli = (int)(tempo_percorrenza_s / dt);
if (offset_cicli >= DEAD_BUF_SIZE) offset_cicli = DEAD_BUF_SIZE - 1;
    if (offset_cicli < 0) offset_cicli = 0;
int read_idx = (dead_buf_idx - offset_cicli) % DEAD_BUF_SIZE;
    if (read_idx < 0) read_idx += DEAD_BUF_SIZE;
    float t_mandata_reale = dead_buffer[read_idx];
dead_buf_idx = (dead_buf_idx + 1) % DEAD_BUF_SIZE;
    
    t_mandata_reale = t_mandata_reale - (dispersioneTubi(t_mandata_reale) / (portata_kgs * cp_fluido));
    t_mandata_reale = max(t_mandata_reale, p_temp_ambiente);
// ── LOGICA STAMPO ──────────────────────────────────────────────
    float Q_conv         = K_stampo_effettivo * (t_mandata_reale - temp_stampo);
    float Q_conv_max     = portata_kgs * cp_fluido * (t_mandata_reale - temp_stampo);
    Q_conv               = constrain(Q_conv, min(0.0f, Q_conv_max), max(0.0f, Q_conv_max));
float Q_disp_stampo  = 4.0f * (temp_stampo - p_temp_ambiente);
    
    // Il processo cede calore anche se il riscaldo è a 0%, purché ci sia circolazione
    float Q_netto_stampo = Q_conv + (circolazione_attiva ? p_pot_stampo : 0.0f) - Q_disp_stampo;
    {
        float delta_T_stampo = (Q_netto_stampo / (p_massa_stampo * CP_ACCIAIO)) * dt;
        temp_stampo += constrain(delta_T_stampo, -5.0f, 5.0f);
    }

    // ── RITORNO E SERBATOIO ────────────────────────────────────────
    float t_ritorno_reale;
    if (portata_kgs < 0.01f) {
        t_ritorno_reale = temp_stampo;
    } else {
        t_ritorno_reale = t_mandata_reale - (Q_conv / (portata_kgs * cp_fluido));
    }
float Q_proc = portata_kgs * cp_fluido * (t_ritorno_reale - temp_serbatoio);
// Calore dissipato meccanicamente dalla pompa nel fluido
    float Q_pompa = circolazione_attiva ? p_pot_pompa : 0.0f;
float pot_netta = Q_nucleo_to_acqua + Q_proc - Q_out + Q_pompa
                    - (p_k_disp_serbatoio * (temp_serbatoio - p_temp_ambiente));
    {
        float delta_T_serb = (pot_netta / (p_massa_acqua * cp_fluido)) * dt;
        temp_serbatoio += constrain(delta_T_serb, -5.0f, 5.0f);
    }

    // ── LIMITI TERMODINAMICI (EBOLLIZIONE E CALORE LATENTE) ────────
    if (temp_serbatoio > t_ebollizione) {
        float Q_ex = p_massa_acqua * cp_fluido * (temp_serbatoio - t_ebollizione);
        float m_persa = Q_ex / 2260000.0f;
        p_massa_acqua -= m_persa;
        if (p_massa_acqua < 0.0f) p_massa_acqua = 0.0f;
        temp_serbatoio = t_ebollizione;
    }

    // ── FILTRO SONDE E USCITA POTENZIOMETRI ───────────────────────
    float alpha_s = ALPHA_SONDA * (SENSOR_DT_MS * 0.001f);
temp_sonda_mandata = filtraPL(temp_sonda_mandata, t_mandata_reale, alpha_s);
    temp_sonda_ritorno = filtraPL(temp_sonda_ritorno, t_ritorno_reale, alpha_s);

    inviaWiper(CS_MANDATA, trovaPassiPotenziometro(tempAResistenza(temp_sonda_mandata + OFFSET_MANDATA)));
    inviaWiper(CS_RITORNO, trovaPassiPotenziometro(tempAResistenza(temp_sonda_ritorno + OFFSET_RITORNO)));
// ── LOG SERIALE ────────────────────────────────────────────────
    if (now - t_log >= 2000UL) {
        t_log = now;
Serial.printf(
            "PMP:%-3s | PWM:%5.1f%% | Serb:%5.1f | Mand:%5.1f Rit:%5.1f | Stmp:%5.1f"
            " | Qhx:%5.0fW Qinj:%5.0fW Qtot:%5.0fW\n",
            stato_pompa ? "ON" : "OFF",
            frac_caldo * 100.0f,
            temp_serbatoio,
            temp_sonda_mandata + OFFSET_MANDATA,
       
 
             temp_sonda_ritorno + OFFSET_RITORNO,
            temp_stampo,
            Q_scambiatore,
            Q_iniezione,
            Q_out
        );
}
}