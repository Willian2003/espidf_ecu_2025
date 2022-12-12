#ifndef PACKETS_H_
#define PACKETS_H_

#include <stdio.h>

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t dps_x;
    int16_t dps_y;
    int16_t dps_z;
} imu_t;

typedef struct
{
    imu_t imu;
    uint16_t rpm;
    uint16_t speed;
    uint8_t temperature;
    uint8_t flags; // MSB - BOX | BUFFER FULL | NC | NC | FUEL_LEVEL | SERVO_ERROR | CHK | RUN - LSB
    uint32_t timestamp;

} packet_t;

// Packet constantly saved
packet_t volatile volatile_packet;

#endif