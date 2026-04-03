#ifndef SYSTEM_H
#define SYSTEM_H

#include <Arduino.h>

struct bytes8_t {
    uint16_t p1;   // Supporta il valore 256
    uint16_t p2;   // Supporta il valore 256
    float r_tot;
};

#define CS_PIN 5
const float R_FISSA = 1004.55;
const float R0_PT1000 = 1000.0;
const float COEFF_PT1000 = 0.003851;

#define TEMPERATURA_INIZIALE 20.0
#define DELTA_T 0.5                  
#define RAFFREDDAMENTO_AL_SECONDO 0.1 
#define TEMPO_ATTIVAZIONE 60   
#define CVD_A 3.9083e-3
#define CVD_B -5.775e-7      

#endif