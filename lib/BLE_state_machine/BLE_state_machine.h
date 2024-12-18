#ifndef BLE_STATE_MACHINE_H
#define BLE_STATE_MACHINE_H

#include <Arduino.h>
#include "packets.h"

void save_can_data(bool CAN_OK, bluetooth *ble);
void save_SD_data(bool SD_OK, bluetooth *ble);
void save_SD_status_data(bluetooth *ble);
void save_SOT_data(uint8_t SOT_OK, bluetooth *ble);
void save_mqtt_client_connection_data(bool mqtt_client_connection, bluetooth *ble);

#endif