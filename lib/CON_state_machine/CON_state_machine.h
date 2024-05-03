#ifndef CON_STATE_MACHINE_H
#define CON_STATE_MACHINE_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include "gprs_defs.h"
#include "hardware_defs.h"
#include "packets.h" 

/* Credentials Variables */
//#define TIM     // Uncomment this line and comment the others if this is your chip
//#define CLARO   // Uncomment this line and comment the others if this is your chip
//#define VIVO    // Uncomment this line and comment the others if this is your chip

/* State Machines */
typedef enum {
    CONNECTED       = 0X01, 
    DISCONNECTED    = 0X00, 
    ERROR_CONECTION = 0x04
} connectivity_states;

uint8_t Initialize_GSM(void);
void gsmCallback(char* topic, byte* payload, unsigned int length);
boolean Check_mqtt_client_conection(void);
uint8_t gsmReconnect(void);
void Send_msg_MQTT(mqtt_packet_t msg_packet);
void publishPacket(void* T, uint32_t len);

/* Ticker functions */
void setup_GSM_tic(void);
/* Interrupts routine */
void ticker1HzISR(void);
void ticker20HzISR(void);

#endif