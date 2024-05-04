#ifndef CAN_H
#define CAN_H

#include <Arduino.h>
#include "src/CANmsg/CANmsg.h"
#include "SD_state_machine.h"
#include "hardware_defs.h"
#include "can_defs.h"
#include "packets.h"

void CAN_setup(void);
void Send_SOT_msg(uint8_t _msg);
mqtt_packet_t update_packet(void);

/* Interrupt */
void canISR(CAN_FRAME* rxMsg);

#endif
