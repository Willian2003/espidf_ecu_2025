#ifndef SOFTWARE_DEFINITIONS_H
#define SOFTWARE_DEFINITIONS_H

#include <Arduino.h>
#include "packets.h"
#include "can_defs.h"
#include "gprs_defs.h"

/* State Machines */
typedef enum {
    CONNECTED       = 0X01, 
    DISCONNECTED    = 0X00, 
    ERROR_CONECTION = 0x04
} connectivity_states;

unsigned long timer;

#endif