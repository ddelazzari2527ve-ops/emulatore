#include <Arduino.h>
#include <SPI.h>
#include "system.h"
#include "math_handling.h"

// ==============================================================
//  PARAMETRI MODIFICABILI DA SERIALE (Valori di Avvio)
// ==============================================================
float p_temp_rete    = 15.0f;    // [T] Temp acqua di rete fredda (°C)
float p_portata      = 20.0f;    // [L] Portata in circolo (L/min)
float p_massa_acqua  = 15.0f;    // [A] Massa d'acqua nel serbatoio interno (Kg)
float p_pot_risc     = 12000.0f; // [R] Potenza Riscaldante Macchina (W)
float p_pot_raff     = 20000.0f; // [F] Potenza Raffreddante Macchina (W)

// CARICO DEL PROCESSO 
float p_pot_stampo   = 0.0f;     // [S] Potenza del processo in Watt (+ scalda, - raffredda)

// ==============================================================
//  COSTANTI E STATO FISICO
// ==============================================================
const float CP_ACQUA   = 4186.0f;  // J/(kg*K) Calore specifico acqua
const float T_AMBIENTE = 20.0f;    // °C dell'officina

float temp_serbatoio = T_AMBIENTE; 

float temp_sonda_mandata = T_AMBIENTE;
float temp_sonda_ritorno = T_AMBIENTE;
float adc_filtrato = 0.0f;

unsigned long t_last = 0;
unsigned long t_log = 0;

// ==============================================================
//  INVIO POTENZIOMETRI
// ==============================================================
void inviaWiper(uint8_t cs, bytes8_t passi) {
    uint8_t w0 = constrain(passi.p1, 0, 255);
    uint8_t w1 = constrain(passi.p2, 0, 255);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    SPI.transfer(0x00); SPI.transfer(w0);
    SPI.transfer(0x10); SPI.transfer(w1);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

// ==============================================================
//  LETTURA SEGNALE MACCHINA
// ==============================================================
float leggiComandoRiscaldo() {
    int raw = analogRead(PIN_CALORE_IN);
    if (raw < ADC_ZERO_SOGLIA) {
        adc_filtrato += ADC_ALPHA * (0.0f - adc_filtrato);
        return 0.0f;
    }
    float frac = (float)raw / ADC_CALDO_MAX;
    frac = constrain(frac, 0.0f, 1.0f);
    adc_filtrato += ADC_ALPHA * (frac - adc_filtrato);
    return adc_filtrato;
}

// ==============================================================
//  MENU E LETTURA SERIALE
// ==============================================================
void stampaMenu() {
    Serial.println("\n========= IMPOSTAZIONI SIMULATORE =========");
    Serial.println("Digita LETTERA + VALORE (es. S5000) e premi INVIO.");
    Serial.printf("[T] Temp Rete    : %.1f °C\n", p_temp_rete);
    Serial.printf("[L] Portata      : %.1f L/min\n", p_portata);
    Serial.printf("[A] Massa Acqua  : %.1f Kg/Litri\n", p_massa_acqua);
    Serial.printf("[R] Pot. Risc.   : %.0f W\n", p_pot_risc);
    Serial.printf("[F] Pot. Raff.   : %.0f W\n", p_pot_raff);
    Serial.printf("[S] Pot. Processo: %.0f W (Positivo = Scalda l'acqua, Negativo = Raffredda)\n", p_pot_stampo);
    Serial.println("=========================================\n");
}

void gestisciSerial() {
    while (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() < 2) {
            if (input == "?") stampaMenu();
            continue;
        }

        char cmd = toupper(input.charAt(0));
        float val = input.substring(1).toFloat();

        switch (cmd) {
            case 'T': p_temp_rete = val;    break;
            case 'L': p_portata = val;      break;
            case 'A': p_massa_acqua = val;  break;
            case 'R': p_pot_risc = val;     break;
            case 'F': p_pot_raff = val;     break;
            case 'S': p_pot_stampo = val;   break;
            case '?': stampaMenu(); break;
        }
        if (cmd != '?') Serial.printf(">>> Parametro %c aggiornato a %.1f\n", cmd, val);
    }
}

// ==============================================================
//  SETUP
// ==============================================================
void setup() {
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);

    pinMode(PIN_RAFFREDDAMENTO, INPUT_PULLUP);
    pinMode(CS_MANDATA, OUTPUT); digitalWrite(CS_MANDATA, HIGH);
    pinMode(CS_RITORNO, OUTPUT); digitalWrite(CS_RITORNO, HIGH);

    SPI.begin();
    stampaMenu();
}

// ==============================================================
//  LOOP FISICO (IL CICLO DELL'ACQUA)
// ==============================================================
void loop() {
    gestisciSerial();

    unsigned long now = millis();
    if (now - t_last < 100) return; 
    float dt = (now - t_last) * 0.001f;
    t_last = now;

    if (p_massa_acqua < 0.1f) p_massa_acqua = 0.1f; 
    float portata_kgs = p_portata / 60.0f;
    if (portata_kgs < 0.01f) portata_kgs = 0.01f;

    // --- 1. LETTURA AZIONI MACCHINA (PID) ---
    float frac_caldo = leggiComandoRiscaldo();
    bool  raffreddo_on = (digitalRead(PIN_RAFFREDDAMENTO) == LOW);

    float Q_in  = frac_caldo * p_pot_risc; 
    float Q_out = 0.0f;                    
    
    if (raffreddo_on && temp_serbatoio > p_temp_rete) {
        Q_out = p_pot_raff;
        float max_q = (temp_serbatoio - p_temp_rete) * p_massa_acqua * CP_ACQUA / dt;
        if (Q_out > max_q) Q_out = max_q; 
    }

    // --- 2. IL PERCORSO DELL'ACQUA (LOOP) ---
    float t_mandata_reale = temp_serbatoio;
    
    // Lo stampo inietta o estrae la potenza S istantaneamente dall'acqua
    float delta_T_stampo = p_pot_stampo / (portata_kgs * CP_ACQUA);
    float t_ritorno_reale = t_mandata_reale + delta_T_stampo;

    // L'acqua rientra nel serbatoio interno portando con sé l'energia acquisita/persa
    float pot_ingresso_ritorno = portata_kgs * CP_ACQUA * (t_ritorno_reale - temp_serbatoio);

    // Bilancio totale del serbatoio: Energia di Ritorno + Resistenze - Scambiatore
    float pot_netta_serbatoio = pot_ingresso_ritorno + Q_in - Q_out;
    temp_serbatoio += (pot_netta_serbatoio / (p_massa_acqua * CP_ACQUA)) * dt;

    // --- 3. INERZIA SONDE E SPI ---
    temp_sonda_mandata += (t_mandata_reale - temp_sonda_mandata) * 0.2f * dt;
    temp_sonda_ritorno += (t_ritorno_reale - temp_sonda_ritorno) * 0.2f * dt;

    inviaWiper(CS_MANDATA, trovaPassiPotenziometro(tempAResistenza(temp_sonda_mandata)));
    inviaWiper(CS_RITORNO, trovaPassiPotenziometro(tempAResistenza(temp_sonda_ritorno)));

    // --- 4. LOG CONSOLE ---
    if (now - t_log >= 2000) {
        t_log = now;
        Serial.printf("MACCHINA:[Risc: %3.0f%% | Raff: %s]  --->  ACQUA:[Mandata: %5.1f°C | Ritorno: %5.1f°C] | Stampo: %+.0fW\n",
                      frac_caldo * 100.0f,
                      raffreddo_on ? "ON " : "OFF",
                      temp_sonda_mandata, 
                      temp_sonda_ritorno,
                      p_pot_stampo);
    }
}