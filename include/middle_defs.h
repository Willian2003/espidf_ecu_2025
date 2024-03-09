#ifndef SOFTWARE_DEFINITIONS_H
#define SOFTWARE_DEFINITIONS_H

#include <Arduino.h>
#include "BajaDefs/hardware_defs.h"
#include "BajaDefs/packets.h"
#include "BajaDefs/gprs_defs.h"
#include "BajaDefs/can_defs.h"

/* State Machines */
typedef enum {
    CONNECTED       = 0X01, 
    DISCONNECTED    = 0X00, 
    ERROR_CONECTION = 0x04
} connectivity_states;

unsigned long timer;

#endif