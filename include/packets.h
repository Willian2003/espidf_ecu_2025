#ifndef PACKETS_H_
#define PACKETS_H_

#include <stdio.h>
#include <string.h>

#define CLEAR(x) memset(&x, 0x00, 8)

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
    int16_t Roll;
    int16_t Pitch;
} Angle_t;

typedef struct
{
    /* BMU DATAS */
    float volt;
    uint8_t SOC;
    uint8_t cvt;
    uint16_t fuel;
    float current;
    /* REAR DATAS */
    uint8_t temperature;
    uint16_t rpm;
    uint8_t flags; // MSB - BOX | BUFFER FULL | NC | NC | FUEL_LEVEL | SERVO_ERROR | CHK | RUN - LSB
    /* FRONT DATAS */
    imu_dps_t imu_dps;
    imu_acc_t imu_acc;
    Angle_t Angle;
    uint16_t speed;
    /* FLAGS */
    uint8_t SOT;   // Connected or not *(0 or 1)
    /* GPS DATAS */
    double latitude;
    double longitude;
    /* DEBUG DATA */
    uint32_t timestamp;

} mqtt_packet_t;

// Packet constantly saved
mqtt_packet_t volatile_packet;

#endif