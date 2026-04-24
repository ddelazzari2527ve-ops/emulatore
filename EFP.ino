#include <Arduino.h>
#include <SPI.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "system.h"
#include "math_handling.h"
#include "table.h"

// ══════════════════════════════════════════════════════════════════
//  PARAMETRI CONFIGURABILI / STATO GLOBALE
// ══════════════════════════════════════════════════════════════════
float p_temp_rete        = 15.0f;
float p_portata          = 20.0f;
float p_massa_acqua      = MASSA_ACQUA_NOMINALE;
float p_pot_risc         = 12000.0f;
float p_pot_raffreddamento = 3000.0f;   // W
float p_k_hx             = 800.0f;      // W/K scambio passivo
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
float t_mandata_stasi    = 20.0f;
float t_ritorno_stasi    = 20.0f;
float t_mandata_reale    = 20.0f;
float t_ritorno_reale    = 20.0f;

float dead_buffer[DEAD_BUF_SIZE] = {0.0f};
int   dead_buf_idx = 0;

bool          errore_acqua         = false;
bool          in_attesa_acqua      = false;
unsigned long t_attesa_acqua       = 0UL;
bool          raffreddamento_cmd   = false;
bool          perdita_attiva       = false;
bool          raffreddamento_attivo = false;
bool          stato_pompa          = false;
PumpMode      pump_mode            = PUMP_MODE_AUTO;
bool          pump_manual_override  = false;
bool          rele_carico_acqua     = false;

bool alarm_boiling                 = false;
bool alarm_dry_run                 = false;
bool alarm_pump_fail               = false;
bool warning_thermal_shock         = false;
bool alarm_circulation_insufficient = false;
bool alarm_serbatoio_vuoto         = false;

unsigned long t_last = 0UL;
unsigned long t_log  = 0UL;

// ══════════════════════════════════════════════════════════════════
//  VARIABILI INTERNE AL MODELLO
// ══════════════════════════════════════════════════════════════════
static float cp_fluido           = CP_ACQUA;
static float t_ebollizione       = 100.0f;
static bool  is_olio             = false;
static float apertura_valvola_hx = 0.0f;
static unsigned long t_ultimo_comando_auto_pompa = 0UL;
static float last_q_scambiatore   = 0.0f;
static float last_q_raffreddamento = 0.0f;
static float last_q_iniezione     = 0.0f;
static float last_q_conv          = 0.0f;
static float last_q_proc          = 0.0f;
static float last_q_pompa         = 0.0f;
static float last_q_out           = 0.0f;

// ══════════════════════════════════════════════════════════════════
//  FUNZIONI DI SUPPORTO
// ══════════════════════════════════════════════════════════════════
static inline float tempoDtSecondi(unsigned long now, unsigned long then) {
    unsigned long diff_ms = now - then;   // overflow-safe su millis()
    float dt_s = diff_ms * 0.001f;
    return (dt_s > DT_MAX_S) ? DT_MAX_S : dt_s;
}

static inline float massaTermicaSerbatoioEffettiva(float massa_acqua) {
    return max(massa_acqua, MASSA_TERMICA_MIN);
}

static inline float portataMassicaDaLMin(float portata_lmin) {
    return max(portata_lmin, 0.0f) / 60.0f;   // 1 L/min ≈ 1 kg/min
}

static inline void aggiornaCalibrazionePT1000(uint8_t cs, float temperatura_c, float offset_ohm) {
    const float r_corretta = resistenzaPt1000Corretta(temperatura_c, offset_ohm);
    const bytes8_t passi   = trovaPassiPotenziometro(r_corretta);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    SPI.transfer((uint8_t)(0x00 | ((passi.p1 > 255U) ? 0x01 : 0x00)));
    SPI.transfer((uint8_t)(passi.p1 & 0xFFU));
    SPI.transfer((uint8_t)(0x10 | ((passi.p2 > 255U) ? 0x01 : 0x00)));
    SPI.transfer((uint8_t)(passi.p2 & 0xFFU));
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

static inline bool startsWithToken(const char *s, const char *token) {
    return (strncmp(s, token, strlen(token)) == 0);
}

static void trimInPlace(char *s) {
    if (!s) return;

    size_t len = strlen(s);
    size_t start = 0;
    while (start < len && isspace((unsigned char)s[start])) start++;

    size_t end = len;
    while (end > start && isspace((unsigned char)s[end - 1])) end--;

    if (start > 0 && end > start) {
        memmove(s, s + start, end - start);
    }
    s[end - start] = '\0';
}

static void toUpperInPlace(char *s) {
    if (!s) return;
    for (; *s; ++s) {
        *s = (char)toupper((unsigned char)*s);
    }
}

static inline float leggiComandoRiscaldo() {
    static bool heat_latched = false;

    const int raw = analogRead(PIN_CALORE_IN);

    if (raw < ADC_ZERO_SOGLIA) {
        adc_filtrato = filtraPL(adc_filtrato, 0.0f, ADC_ALPHA);
        heat_latched = false;
        return 0.0f;
    }

    float frazione = constrain((float)raw / ADC_CALDO_MAX, 0.0f, 1.0f);
    adc_filtrato = filtraPL(adc_filtrato, frazione, ADC_ALPHA);

    if (!heat_latched) {
        if (adc_filtrato >= HEAT_ON_TH) {
            heat_latched = true;
        }
    } else {
        if (adc_filtrato <= HEAT_OFF_TH) {
            heat_latched = false;
        }
    }

    return heat_latched ? adc_filtrato : 0.0f;
}

static void stampaAllarmiSintetici() {
    bool vuoto = true;
    Serial.print(F("ALLARMI: "));
    if (alarm_boiling) {
        Serial.print(F("Bollitura"));
        vuoto = false;
    }
    if (alarm_dry_run) {
        if (!vuoto) Serial.print(F(" | "));
        Serial.print(F("Secco"));
        vuoto = false;
    }
    if (alarm_pump_fail) {
        if (!vuoto) Serial.print(F(" | "));
        Serial.print(F("Guasto pompa"));
        vuoto = false;
    }
    if (warning_thermal_shock) {
        if (!vuoto) Serial.print(F(" | "));
        Serial.print(F("Shock termico"));
        vuoto = false;
    }
    if (alarm_circulation_insufficient) {
        if (!vuoto) Serial.print(F(" | "));
        Serial.print(F("Circolazione insufficiente"));
        vuoto = false;
    }
    if (alarm_serbatoio_vuoto) {
        if (!vuoto) Serial.print(F(" | "));
        Serial.print(F("Serbatoio vuoto"));
        vuoto = false;
    }

    if (vuoto) Serial.print(F("Nessuno"));
    Serial.println();
}

static void stampaStatoSintetico() {
    const char* modo_pompa = (pump_mode == PUMP_MODE_AUTO) ? "AUTO" :
                             (pump_mode == PUMP_MODE_MANUAL_ON ? "MAN-ON" : "MAN-OFF");
    const bool richiesta_freddo = (raffreddamento_cmd || raffreddamento_attivo);
    const bool pompa_effettiva = (stato_pompa && p_portata > 0.0f && !alarm_serbatoio_vuoto);

    Serial.printf("POMPA:%s [%s] | FREDDO:%s | RIEMPIMENTO:%s | POT.RAFF:%6.0f W\n",
                  stato_pompa ? "ON" : "OFF",
                  modo_pompa,
                  richiesta_freddo ? "ON" : "OFF",
                  rele_carico_acqua ? "ON" : "OFF",
                  p_pot_raffreddamento);

    Serial.printf("TERMICO | SERB:%6.2f C | MAND:%6.2f C | RIT:%6.2f C | STAMPO:%6.2f C | RES:%6.2f C | NUCLEO:%6.2f C\n",
                  temp_serbatoio,
                  t_mandata_reale,
                  t_ritorno_reale,
                  temp_stampo,
                  temp_resistenza,
                  temp_nucleo_res);

    Serial.printf("FLUSSO  | PORTATA:%5.1f L/min | MASSA ACQUA:%5.2f kg | MASSA STAMPO:%5.2f kg | POMPA EFF:%s\n",
                  p_portata,
                  p_massa_acqua,
                  p_massa_stampo,
                  pompa_effettiva ? "SI" : "NO");

    Serial.printf("ENERGIA | Qhx:%7.0f W | Qraff:%7.0f W | Qfill:%7.0f W | Qconv:%7.0f W | Qproc:%7.0f W | Qpump:%7.0f W\n",
                  last_q_scambiatore,
                  last_q_raffreddamento,
                  last_q_iniezione,
                  last_q_conv,
                  last_q_proc,
                  last_q_pompa);

    stampaAllarmiSintetici();
}

static void stampaMenu() {
    Serial.println(F("=============== SIMULATORE TERMICO ==============="));
    Serial.println(F("Comandi rapidi:"));
    Serial.println(F("  MENU o ?           -> mostra questo menu"));
    Serial.println(F("  STATO              -> stampa riepilogo immediato"));
    Serial.println(F("  POMPA AUTO         -> automatico"));
    Serial.println(F("  POMPA ON / P1      -> pompa manuale ON"));
    Serial.println(F("  POMPA OFF / P0     -> pompa manuale OFF"));
    Serial.println(F("  RAFFREDDAMENTO     -> abilita richiesta freddo"));
    Serial.println(F("  STOP               -> disabilita richiesta freddo"));
    Serial.println(F("  OLIO / H2O         -> selezione fluido"));
    Serial.println(F("  ACQUA              -> avvio riempimento manuale"));
    Serial.println(F("  PERDITA            -> simula svuotamento"));
    Serial.println(F("  RIPRISTINA         -> reset completo dello stato"));
    Serial.println(F("Parametri numerici (esempio: F3000 oppure T45.0):"));
    Serial.println(F("  T rete [C]"));
    Serial.println(F("  E ambiente [C]"));
    Serial.println(F("  L portata [L/min]"));
    Serial.println(F("  A massa acqua [kg]"));
    Serial.println(F("  M massa stampo [kg]"));
    Serial.println(F("  R potenza resistenza [W]"));
    Serial.println(F("  F potenza raffreddamento [W]"));
    Serial.println(F("  S carico stampo [W]"));
    Serial.println(F("  X dispersione tubi [W/K]"));
    Serial.println(F("  Y dispersione serbatoio [W/K]"));
    Serial.println(F("  V portata valvola riempimento [L/min]"));
    Serial.println(F("--------------------------------------------------"));
    Serial.printf("Valori | T rete:%5.1f C | T amb:%5.1f C | Portata:%5.1f L/min | Acqua:%5.2f kg | Raffredd.: %6.0f W\n",
                  p_temp_rete, p_temp_ambiente, p_portata, p_massa_acqua, p_pot_raffreddamento);
    Serial.printf("Impianto | Res:%7.0f W | HX:%6.1f W/K | Stampo:%6.0f W | Valvola:%4.1f L/min\n",
                  p_pot_risc, p_k_hx, p_pot_stampo, p_portata_valvola);
    stampaAllarmiSintetici();
    Serial.println(F("=================================================="));
}

static void resetStatoTermico() {
    const float t0 = p_temp_ambiente;

    temp_resistenza    = t0;
    temp_nucleo_res    = t0;
    temp_serbatoio     = t0;
    temp_sonda_mandata = t0;
    temp_sonda_ritorno = t0;
    temp_stampo        = t0;
    t_mandata_stasi    = t0;
    t_ritorno_stasi    = t0;
    t_mandata_reale    = t0;
    t_ritorno_reale    = t0;

    p_massa_acqua = MASSA_ACQUA_NOMINALE;

    for (int i = 0; i < DEAD_BUF_SIZE; ++i) {
        dead_buffer[i] = t0;
    }
    dead_buf_idx = 0;

    adc_filtrato = 0.0f;
    apertura_valvola_hx = 0.0f;
    pump_mode = PUMP_MODE_AUTO;
    pump_manual_override = false;
    stato_pompa = false;

    errore_acqua = false;
    in_attesa_acqua = false;
    t_attesa_acqua = 0UL;
    raffreddamento_cmd = false;
    perdita_attiva = false;
    raffreddamento_attivo = false;
    rele_carico_acqua = false;

    alarm_boiling = false;
    alarm_dry_run = false;
    alarm_pump_fail = false;
    warning_thermal_shock = false;
    alarm_circulation_insufficient = false;
    alarm_serbatoio_vuoto = false;
}

static void emettiCambioAllarme(bool nuovo_stato, bool &stato_memoria, const __FlashStringHelper *on_msg, const __FlashStringHelper *off_msg) {
    if (nuovo_stato == stato_memoria) return;
    stato_memoria = nuovo_stato;
    Serial.println(nuovo_stato ? on_msg : off_msg);
}

static void gestisciSerial() {
    if (!Serial.available()) return;

    char input[96];
    size_t n = Serial.readBytesUntil('\n', input, sizeof(input) - 1);
    input[n] = '\0';
    trimInPlace(input);
    if (input[0] == '\0') return;

    toUpperInPlace(input);

    if (strcmp(input, "?") == 0 || strcmp(input, "MENU") == 0 || strcmp(input, "AIUTO") == 0) {
        stampaMenu();
        return;
    }

    if (strcmp(input, "STATO") == 0) {
        stampaStatoSintetico();
        return;
    }

    if (strcmp(input, "ACQUA") == 0 || strcmp(input, "REINTEGRA") == 0) {
        errore_acqua = true;
        in_attesa_acqua = true;
        t_attesa_acqua = millis();
        rele_carico_acqua = true;
        digitalWrite(PIN_RELE_ACQUA, LOW);   // relè attivo LOW
        Serial.println(F("OK: riempimento acqua avviato"));
        return;
    }

    if (strcmp(input, "PERDITA") == 0) {
        errore_acqua = true;
        perdita_attiva = true;
        Serial.println(F("OK: simulazione perdita attiva"));
        return;
    }

    if (startsWithToken(input, "RAFFREDDAMENTO")) {
        raffreddamento_attivo = true;
        Serial.println(F("OK: richiesta freddo attiva"));
        return;
    }

    if (strcmp(input, "STOP") == 0 || startsWithToken(input, "FREDDO OFF")) {
        raffreddamento_attivo = false;
        Serial.println(F("OK: richiesta freddo disattivata"));
        return;
    }

    if (strcmp(input, "RIPRISTINA") == 0 || strcmp(input, "RESET") == 0) {
        resetStatoTermico();
        digitalWrite(PIN_RELE_ACQUA, HIGH);
        Serial.println(F("OK: stato ripristinato"));
        return;
    }

    if (strcmp(input, "POMPA AUTO") == 0 || strcmp(input, "PAUTO") == 0) {
        pump_manual_override = false;
        pump_mode = PUMP_MODE_AUTO;
        Serial.println(F("OK: pompa in automatico"));
        return;
    }

    if (strcmp(input, "POMPA ON") == 0 || strcmp(input, "P1") == 0) {
        pump_manual_override = true;
        pump_mode = PUMP_MODE_MANUAL_ON;
        stato_pompa = true;
        t_ultimo_comando_auto_pompa = millis();
        Serial.println(F("OK: pompa ON manuale"));
        return;
    }

    if (strcmp(input, "POMPA OFF") == 0 || strcmp(input, "P0") == 0) {
        pump_manual_override = true;
        pump_mode = PUMP_MODE_MANUAL_OFF;
        stato_pompa = false;
        Serial.println(F("OK: pompa OFF manuale"));
        return;
    }

    if (strcmp(input, "OLIO") == 0) {
        is_olio = true;
        cp_fluido = CP_OLIO_VAL;
        t_ebollizione = 300.0f;
        Serial.println(F("OK: fluido impostato su OLIO"));
        return;
    }

    if (strcmp(input, "H2O") == 0) {
        is_olio = false;
        cp_fluido = CP_ACQUA;
        t_ebollizione = 100.0f;
        Serial.println(F("OK: fluido impostato su ACQUA"));
        return;
    }

    const char lettera = input[0];
    if (!isalpha((unsigned char)lettera)) {
        Serial.println(F("Comando non riconosciuto. Digita MENU o ?"));
        return;
    }

    float val = strtof(input + 1, nullptr);

    switch (lettera) {
        case 'T': p_temp_rete          = clampTempRealistica(val);                        break;
        case 'E': p_temp_ambiente      = clampTempRealistica(val);                        break;
        case 'L': p_portata            = max(0.0f, val);                                  break;
        case 'A': p_massa_acqua        = max(0.0f, val);                                  break;
        case 'M': p_massa_stampo       = max(0.1f, val);                                  break;
        case 'X': p_k_disp_tubi        = max(0.0f, val);                                  break;
        case 'Y': p_k_disp_serbatoio   = max(0.0f, val);                                  break;
        case 'R': p_pot_risc           = constrain(val, 0.0f, POTENZA_RISCALDO_MAX_W);    break;
        case 'F': p_pot_raffreddamento = constrain(val, 0.0f, POTENZA_RAFFREDDAMENTO_MAX_W); break;
        case 'S': p_pot_stampo         = val;                                             break;
        case 'V': p_portata_valvola    = constrain(val, 0.0f, PORTATA_VALVOLA_MAX_LMIN);  break;
        default:
            Serial.println(F("Comando non riconosciuto. Digita MENU o ?"));
            break;
    }
}

static void aggiornaAllarmiRealtime(float t_mandata_locale,
                                    float t_ritorno_locale,
                                    float flusso_lmin,
                                    bool richiesta_freddo,
                                    bool pompa_effettiva) {
    const bool new_boiling = (temp_serbatoio >= t_ebollizione);
    emettiCambioAllarme(new_boiling, alarm_boiling,
                        F("ALLARME: bollitura"), F("RIPRISTINO: bollitura"));

    const bool new_dry = (p_massa_acqua < MASSA_CRITICA_ACQUA);
    emettiCambioAllarme(new_dry, alarm_dry_run,
                        F("ALLARME: secco"), F("RIPRISTINO: secco"));

    const bool new_circ_insuff = (pompa_effettiva && flusso_lmin > 0.0f && flusso_lmin < PORTATA_MIN_CIRC_LMIN);
    emettiCambioAllarme(new_circ_insuff, alarm_circulation_insufficient,
                        F("ALLARME: circolazione insufficiente"), F("RIPRISTINO: circolazione insufficiente"));

    const bool new_pump_fail = (richiesta_freddo && pompa_effettiva && flusso_lmin <= 0.0f);
    emettiCambioAllarme(new_pump_fail, alarm_pump_fail,
                        F("ALLARME: guasto pompa"), F("RIPRISTINO: guasto pompa"));

    const bool new_shock = (fabsf(t_mandata_locale - t_ritorno_locale) > THERMAL_SHOCK_DELTA_C);
    emettiCambioAllarme(new_shock, warning_thermal_shock,
                        F("ALLARME: shock termico"), F("RIPRISTINO: shock termico"));
}

static float aggiornaDeadTimeMandata(float t_sorgente, float flusso_lmin, float dt_s) {
    static float last_out = 20.0f;
    static float accum_s = 0.0f;

    if (dt_s <= 0.0f || flusso_lmin <= 0.0f) {
        return last_out;   // nessun avanzamento energia quando la portata è zero
    }

    accum_s += dt_s;

    float delay_s = DEADTIME_S * (PORTATA_NOMINALE_LMIN / max(flusso_lmin, 0.01f));
    delay_s = clampFloat(delay_s, CICLO_S, DEADTIME_MAX_S);

    int delay_samples = (int)lroundf(delay_s / CICLO_S);
    if (delay_samples < 1) delay_samples = 1;
    if (delay_samples >= DEAD_BUF_SIZE) {
        delay_samples = DEAD_BUF_SIZE - 1;
        alarm_circulation_insufficient = true;   // non saturare silenziosamente
    }

    int push_count = (int)floorf(accum_s / CICLO_S);
    if (push_count < 1) push_count = 1;
    accum_s -= (float)push_count * CICLO_S;
    if (accum_s < 0.0f) accum_s = 0.0f;

    for (int i = 0; i < push_count; ++i) {
        dead_buffer[dead_buf_idx] = t_sorgente;
        dead_buf_idx = (dead_buf_idx + 1) % DEAD_BUF_SIZE;
    }

    int read_idx = dead_buf_idx - delay_samples;
    while (read_idx < 0) read_idx += DEAD_BUF_SIZE;

    last_out = dead_buffer[read_idx];
    return last_out;
}

static void aggiornaStatoPompa(unsigned long now_ms, float frac_caldo, bool richiesta_freddo) {
    (void)richiesta_freddo; // la pompa non deve accendersi solo per il freddo

    if (pump_manual_override) {
        if (pump_mode == PUMP_MODE_MANUAL_ON)  stato_pompa = true;
        if (pump_mode == PUMP_MODE_MANUAL_OFF) stato_pompa = false;
        return;
    }

    pump_mode = PUMP_MODE_AUTO;

    if (frac_caldo >= HEAT_ON_TH) {
        stato_pompa = true;
        t_ultimo_comando_auto_pompa = now_ms;
        return;
    }

    if (stato_pompa && (now_ms - t_ultimo_comando_auto_pompa >= 15000UL)) {
        stato_pompa = false;
    }
}

static void aggiornaRifornimentoAcqua(unsigned long now_ms) {
    if (!alarm_serbatoio_vuoto && p_massa_acqua <= MASSA_SERBATOIO_VUOTO_ON_KG) {
        alarm_serbatoio_vuoto = true;
        rele_carico_acqua = true;
        errore_acqua = true;
        in_attesa_acqua = true;
        t_attesa_acqua = now_ms;
        Serial.println(F("ALLARME: serbatoio vuoto -> riempimento attivo"));
    }

    if (alarm_serbatoio_vuoto && p_massa_acqua >= MASSA_SERBATOIO_VUOTO_OFF_KG) {
        alarm_serbatoio_vuoto = false;
        rele_carico_acqua = false;
        errore_acqua = false;
        in_attesa_acqua = false;
        Serial.println(F("RIPRISTINO: serbatoio riempito"));
    }

    digitalWrite(PIN_RELE_ACQUA, rele_carico_acqua ? LOW : HIGH); // relè attivo LOW
}

// ══════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ══════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);

#ifdef ARDUINO_ARCH_ESP32
    analogSetAttenuation(ADC_11db);
#endif

    pinMode(PIN_RAFFREDDAMENTO, INPUT_PULLUP);
    pinMode(CS_MANDATA, OUTPUT);
    pinMode(CS_RITORNO, OUTPUT);
    pinMode(PIN_RELE_ACQUA, OUTPUT);

    digitalWrite(CS_MANDATA, HIGH);
    digitalWrite(CS_RITORNO, HIGH);
    digitalWrite(PIN_RELE_ACQUA, HIGH);

    SPI.begin();

    resetStatoTermico();
    t_last = millis();
    t_log  = t_last;
    t_ultimo_comando_auto_pompa = t_last;

    stampaMenu();
}

void loop() {
    gestisciSerial();
    unsigned long now = millis();

    // Richiesta freddo da ingresso hardware, attivo LOW.
    raffreddamento_cmd = (digitalRead(PIN_RAFFREDDAMENTO) == LOW);

    if (now - t_last < CICLO_MS) return;
    float dt_s = tempoDtSecondi(now, t_last);
    t_last = now;

    // Portata reale disponibile.
    float flusso_lmin = max(p_portata, 0.0f);
    float portata_kgs  = portataMassicaDaLMin(flusso_lmin);

    // Gestione rifornimento automatico.
    aggiornaRifornimentoAcqua(now);

    // Rifornimento / perdita simulata.
    float Q_iniezione = 0.0f;
    if (rele_carico_acqua) {
        float portata_fill_kgs = constrain(p_portata_valvola, 0.0f, PORTATA_VALVOLA_MAX_LMIN) / 60.0f;
        Q_iniezione = portata_fill_kgs * cp_fluido * (p_temp_ambiente - temp_serbatoio);
        p_massa_acqua += portata_fill_kgs * dt_s;
        if (p_massa_acqua > MASSA_ACQUA_NOMINALE) p_massa_acqua = MASSA_ACQUA_NOMINALE;
    } else if (perdita_attiva) {
        float portata_perdita_kgs = PORTATA_PERDITA_LMIN / 60.0f;
        p_massa_acqua -= portata_perdita_kgs * dt_s;
        if (p_massa_acqua < 0.0f) p_massa_acqua = 0.0f;
    }

    // Comando caldo: filtro e isteresi.
    float frac_caldo_raw = leggiComandoRiscaldo();

    // Richiesta freddo da hardware o da comando seriale.
    bool richiesta_freddo = raffreddamento_cmd || raffreddamento_attivo;

    // Stato pompa: manuale assoluto, altrimenti automatico.
    aggiornaStatoPompa(now, frac_caldo_raw, richiesta_freddo);

    // Se il serbatoio è vuoto, blocco il circuito di raffreddamento e scambio.
    bool circuito_idraulico_attivo = (stato_pompa && flusso_lmin > 0.0f && !alarm_serbatoio_vuoto && p_massa_acqua > 0.0f);

    // Feed-forward sul riscaldatore.
    float Q_ff = -ff_gain * p_pot_stampo * (p_portata / PORTATA_NOMINALE_LMIN);
    float frac_caldo = constrain(frac_caldo_raw + Q_ff / max(p_pot_risc, 1.0f), 0.0f, 1.0f);
    if (richiesta_freddo) frac_caldo = 0.0f;
    float Q_in_elettrico = potenzaRiscaldo(frac_caldo, p_pot_risc);

    // ══════════════════════════════════════════════════════════════
    //  MODELLO RESISTENZA + NUCLEO
    // ══════════════════════════════════════════════════════════════
    float C_res = MASSA_RES_KG * CP_ACCIAIO;
    float Q_res_to_nucleo = K_SCAMBIO_RES * (temp_resistenza - temp_nucleo_res);

    float Q_loss_dry = 0.0f;
    if (p_massa_acqua < MASSA_CRITICA_ACQUA) {
        Q_loss_dry = K_RES_DRY * max(temp_resistenza - p_temp_ambiente, 0.0f);
    }

    temp_resistenza += ((Q_in_elettrico - Q_res_to_nucleo - Q_loss_dry) / max(C_res, 1.0f)) * dt_s;

    float wet_factor = clampFloat(p_massa_acqua / MASSA_CRITICA_ACQUA, 0.0f, 1.0f);
    float Q_nucleo_to_acqua = K_NUCLEO * (temp_nucleo_res - temp_serbatoio) * wet_factor;
    float C_nucleo = M_NUCLEO * CP_ACCIAIO;
    temp_nucleo_res += ((Q_res_to_nucleo - Q_nucleo_to_acqua) / max(C_nucleo, 1.0f)) * dt_s;

    // ══════════════════════════════════════════════════════════════
    //  SCAMBI TERMICI / RAFFREDDAMENTO
    // ══════════════════════════════════════════════════════════════
    float target_apertura = richiesta_freddo ? 1.0f : 0.0f;
    apertura_valvola_hx = filtraPL(apertura_valvola_hx, target_apertura, alphaDaTau(dt_s, 3.0f));

    float rapporto_portata = max(flusso_lmin / PORTATA_NOMINALE_LMIN, 0.01f);

    float Q_scambiatore = 0.0f;
    float Q_raffreddamento = 0.0f;
    if (circuito_idraulico_attivo) {
        float K_hx_effettivo = p_k_hx * powf(rapporto_portata, 0.8f);
        Q_scambiatore = potenzaScambiatore(true, temp_serbatoio, p_temp_rete, K_hx_effettivo) * apertura_valvola_hx;

        // Raffreddamento attivo limitato da potenza in Watt e dalla portata reale.
        float Q_fisico = potenzaRaffreddamento(true, temp_serbatoio, p_temp_rete, portata_kgs);
        Q_raffreddamento = min(Q_fisico, p_pot_raffreddamento);
    }

    // Mandata: il dead-time avanza solo con portata reale.
    if (circuito_idraulico_attivo) {
        t_mandata_reale = aggiornaDeadTimeMandata(temp_serbatoio, flusso_lmin, dt_s);
        t_mandata_stasi = t_mandata_reale;
    } else {
        t_mandata_stasi = filtraPL(t_mandata_stasi, p_temp_ambiente, alphaDaTau(dt_s, 12.0f));
        t_mandata_reale = t_mandata_stasi;
    }

    // Stampo industriale.
    float Q_conv = 0.0f;
    if (circuito_idraulico_attivo) {
        float K_stampo_effettivo = K_stampo * powf(rapporto_portata, 0.8f);
        Q_conv = K_stampo_effettivo * (t_mandata_reale - temp_stampo);

        float Q_conv_lim = portata_kgs * cp_fluido * fabsf(t_mandata_reale - temp_stampo);
        Q_conv = clampFloat(Q_conv, -Q_conv_lim, Q_conv_lim);
    }

    float Q_disp_stampo = 4.0f * (temp_stampo - p_temp_ambiente);
    float Q_netto_stampo = Q_conv + (circuito_idraulico_attivo ? p_pot_stampo : 0.0f) - Q_disp_stampo;
    float C_stampo = max(p_massa_stampo * CP_ACCIAIO, 1.0f);
    temp_stampo += (Q_netto_stampo / C_stampo) * dt_s;

    // Ritorno.
    if (circuito_idraulico_attivo) {
        float deltaT_ritorno = Q_conv / max(portata_kgs * cp_fluido, 1.0f);
        t_ritorno_reale = t_mandata_reale - deltaT_ritorno;
        t_ritorno_stasi = t_ritorno_reale;
    } else {
        t_ritorno_stasi = filtraPL(t_ritorno_stasi, temp_stampo, alphaDaTau(dt_s, 12.0f));
        t_ritorno_reale = t_ritorno_stasi;
    }

    // Serbatoio: bilancio energetico coerente.
    float Q_proc = circuito_idraulico_attivo ? (portata_kgs * cp_fluido * (t_ritorno_reale - temp_serbatoio)) : 0.0f;
    float Q_pompa = circuito_idraulico_attivo ? PUMP_HEAT_W : 0.0f;

    last_q_scambiatore     = Q_scambiatore;
    last_q_raffreddamento  = Q_raffreddamento;
    last_q_iniezione       = Q_iniezione;
    last_q_conv            = Q_conv;
    last_q_proc            = Q_proc;
    last_q_pompa           = Q_pompa;
    last_q_out             = Q_scambiatore + Q_raffreddamento;

    float C_serbatoio = massaTermicaSerbatoioEffettiva(p_massa_acqua) * cp_fluido;
    float pot_netta = Q_nucleo_to_acqua + Q_proc - Q_scambiatore - Q_raffreddamento + Q_iniezione + Q_pompa
                      - (p_k_disp_serbatoio * (temp_serbatoio - p_temp_ambiente));
    temp_serbatoio += (pot_netta / max(C_serbatoio, 1.0f)) * dt_s;

    // ══════════════════════════════════════════════════════════════
    //  EBOLLIZIONE / TRASFORMAZIONE DI FASE
    // ══════════════════════════════════════════════════════════════
    if (p_massa_acqua > 0.0f && temp_serbatoio > t_ebollizione) {
        float Q_ex = p_massa_acqua * cp_fluido * (temp_serbatoio - t_ebollizione);
        float m_persa = Q_ex / 2260000.0f;
        float m_persa_max = 0.03f * dt_s;   // rate limit evaporazione
        if (m_persa > m_persa_max) m_persa = m_persa_max;
        p_massa_acqua -= m_persa;
        if (p_massa_acqua < 0.0f) p_massa_acqua = 0.0f;
        temp_serbatoio = t_ebollizione;
    }

    // Evito derive numeriche.
    temp_resistenza    = clampTempRealistica(temp_resistenza);
    temp_nucleo_res    = clampTempRealistica(temp_nucleo_res);
    temp_serbatoio     = clampTempRealistica(temp_serbatoio);
    temp_sonda_mandata = clampTempRealistica(temp_sonda_mandata);
    temp_sonda_ritorno = clampTempRealistica(temp_sonda_ritorno);
    temp_stampo        = clampTempRealistica(temp_stampo);
    t_mandata_reale    = clampTempRealistica(t_mandata_reale);
    t_ritorno_reale    = clampTempRealistica(t_ritorno_reale);
    t_mandata_stasi    = clampTempRealistica(t_mandata_stasi);
    t_ritorno_stasi    = clampTempRealistica(t_ritorno_stasi);

    if (p_massa_acqua < 0.0f) p_massa_acqua = 0.0f;
    if (p_massa_acqua > MASSA_ACQUA_NOMINALE) p_massa_acqua = MASSA_ACQUA_NOMINALE;

    // Filtri sonde PT1000 e uscita SPI.
    float alpha_s = alphaDaTau(dt_s, SONDA_TAU_S);
    temp_sonda_mandata = filtraPL(temp_sonda_mandata, t_mandata_reale, alpha_s);
    temp_sonda_ritorno  = filtraPL(temp_sonda_ritorno, t_ritorno_reale, alpha_s);

    aggiornaCalibrazionePT1000(CS_MANDATA, temp_sonda_mandata, R_OFFSET_MANDATA_OHM);
    aggiornaCalibrazionePT1000(CS_RITORNO, temp_sonda_ritorno, R_OFFSET_RITORNO_OHM);

    // Allarmi.
    aggiornaAllarmiRealtime(t_mandata_reale, t_ritorno_reale, flusso_lmin, richiesta_freddo, circuito_idraulico_attivo);

    // Aggiorna lo stato del rifornimento dopo il calcolo.
    aggiornaRifornimentoAcqua(now);

    // ══════════════════════════════════════════════════════════════
    //  LOG SERIALE PERIODICO
    // ══════════════════════════════════════════════════════════════
    if (now - t_log >= 2000UL) {
        t_log = now;
 Serial.printf(
    "POMPA:%s | FREDDO:%s | RIEMP:%s | SERB:%6.2f C | MAND:%6.2f C | RIT:%6.2f C | STAMPO:%6.2f C | ACQUA:%5.2f kg\n",
    stato_pompa ? "ON" : "OFF",
    richiesta_freddo ? "ON" : "OFF",
    rele_carico_acqua ? "ON" : "OFF",
    temp_serbatoio,
    t_mandata_reale,
    t_ritorno_reale,
    temp_stampo,
    p_massa_acqua
);

        Serial.printf(
            "Qhx:%7.0f W | Qraff:%7.0f W | Qfill:%7.0f W | Qconv:%7.0f W | Qproc:%7.0f W | Qpump:%7.0f W",
            last_q_scambiatore,
            last_q_raffreddamento,
            last_q_iniezione,
            last_q_conv,
            last_q_proc,
            last_q_pompa
        );

        stampaAllarmiSintetici();
    }
}
