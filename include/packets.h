#ifndef PACKETS_H
#define PACKETS_H

#include <stdio.h>
#include <string.h>

#define SUCESS_RESPONSE 2
#define FAIL_RESPONSE   1

typedef struct
{
    int16_t acc_x = 10;
    int16_t acc_y = 12;
    int16_t acc_z = 13;
} imu_acc_t;

typedef struct
{
    int16_t dps_x = 14 ;
    int16_t dps_y = 15;
    int16_t dps_z = 16;
} imu_dps_t;

typedef struct
{
    int16_t Roll = 17;
    int16_t Pitch = 18;
} Angle_t;

typedef struct
{
    /* REAR DATAS */
    float volt = 18.5;
    uint8_t SOC = 19;
    uint8_t cvt = 20;
    // uint16_t fuel;
    float current = 21.5;
    uint8_t temperature = 22;
    uint16_t speed = 23;

    /* FRONT DATAS */
    imu_acc_t imu_acc;
    imu_dps_t imu_dps;
    Angle_t Angle;
    uint16_t rpm = 4800;
    uint8_t flags = 8; // MSB - BOX | BUFFER FULL | NC | NC | FUEL_LEVEL | SERVO_ERROR | CHK | RUN - LSB

    /* MPU DATAS */
    double latitude = 24.45;
    double longitude = 25.45;

    /* DEBUG DATA */
    uint32_t timestamp = 2640;

} mqtt_packet_t;

typedef struct
{
    // MPU_Bluetooth (sent by physical serial connection)
    //  String config_bluetooth_enabled;
    //  String config_bluedroid_enabled;
    //  String config_bt_spp_enabled;

    // MPU
    uint8_t lora_init;

    // SCU
    uint8_t internet_modem;         // 0
    uint8_t mqtt_client_connection; // 1
    uint8_t sd_start;               // 2
    uint8_t check_sd;               // 3

    // FRONT
    uint8_t accel_begin;

    // REAR
    uint8_t termistor;
    uint8_t cvt_temperature;
    uint8_t measure_volt;
    uint8_t speed_pulse_counter;
    uint16_t servo_state;

} bluetooth;

#endif
