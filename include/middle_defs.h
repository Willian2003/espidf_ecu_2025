#ifndef SOFTWARE_DEFINITIONS_H_
#define SOFTWARE_DEFINITIONS_H_

#include <Arduino.h>

/* State Machines */
typedef enum {IDLE_ST, SOT_ST, DEBUG_ST} state_t;

typedef enum{CONNECTED=0X01, DISCONNECTED=0X00} connectivity_states;

unsigned long timer;

#endif