#ifndef HARDWARE_DEFS_H_
#define HARDWARE_DEFS_H_

/* pinout definitions */
// LEDS
#define EMBEDDED_LED LED_BUILTIN // Turned on on failiure
#define DEBUG_LED    25

/* SD */ 
#define SD_CS  5 
// SPI BUS
// VSPI
#define MISO   19
#define MOSI   23
#define SCK    18

/* CAN PINS */ 
#define CAN_TX 21  
#define CAN_RX 22
// MCP2515
#define CAN_CS        4
#define CAN_INTERRUPT 22

/* GPRS */
#define MODEM_RST 
#define GPRS_TX   17
#define GPRS_RX   16

#endif