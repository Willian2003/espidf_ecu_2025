#ifndef PACKETS_H_
#define PACKETS_H_

#include <stdio.h>

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
} imu_acc_t;

typedef struct
{
    int16_t dps_x;
    int16_t dps_y;
    int16_t dps_z;
} imu_dps_t;

typedef struct
{
    /* BMU DATAS */
    double volt;
    uint8_t soc;
    uint8_t cvt;
    uint16_t fuel;
    double current;
    /* REAR DATAS */
    uint8_t temperature;
    uint8_t flags; // MSB - BOX | BUFFER FULL | NC | NC | FUEL_LEVEL | SERVO_ERROR | CHK | RUN - LSB
    uint16_t rpm;
    /* FRONT DATAS */
    imu_dps_t imu_dps;
    imu_acc_t imu_acc;
    uint16_t speed;
    /* SCU DATAS */
    uint8_t SOT;  //State of Telemetry, only for electronic panel
    double latitude;
    double longitude;
    /* DEBUG DATA */
    uint32_t timestamp;

} packet_t;

// Packet constantly saved
packet_t volatile_packet;

#endif