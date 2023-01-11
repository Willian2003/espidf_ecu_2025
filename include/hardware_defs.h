#ifndef HARDWARE_DEFS_H_
#define HARDWARE_DEFS_H_


/* pinout definitions */
// LEDS c c c    v
#define EMBEDDED_LED 2 // Turned on on failiure
#define DEBUG_LED 15

// SD
#define SD_CS 5 

// CAN PINS
#define CAN_CS 4
#define CAN_INTERRUPT 22

// SPI BUS
// VSPI
#define MISO 19
#define MOSI 23
#define SCK 18

//GPRS
#define MODEM_RST 33

//GPS
#define GPSTX 12
#define GPSRX 13

#endif