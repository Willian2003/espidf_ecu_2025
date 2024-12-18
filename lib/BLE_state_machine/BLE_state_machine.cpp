#include <BLE_state_machine.h>

void save_SD_data(bool SD_OK, bluetooth *ble)
{
    if (SD_OK)
        ble->sd_start = 2;
    else
        ble->sd_start = 1;

    Serial.print("_sd --> ");
    Serial.println(ble->sd_start);
}

void save_SD_status_data(bluetooth *ble)
{
    Serial.println("Out the function");
    Serial.print("bluetooth_packet.check_sd --> ");
    Serial.println(ble->check_sd);
}

void save_SOT_data(uint8_t SOT_OK, bluetooth *ble)
{
    if (SOT_OK == 0x04 /*ERROR_CONECTION*/)
    {
        ble->internet_modem = 1;
        Serial.print("internet_modem_erro --> ");
        Serial.println(ble->internet_modem);
    }

    else
    {
        ble->internet_modem = 2;
        Serial.print("internet_modem_ok --> ");
        Serial.println(ble->internet_modem);
    }

    Serial.print("internet_modem_geral --> ");
    Serial.println(ble->internet_modem);
}

void save_mqtt_client_connection_data(bool mqtt_client_connection, bluetooth *ble)
{
    if (!mqtt_client_connection)
    {
        ble->mqtt_client_connection = 1;
        Serial.print("mqtt_client_connection_erro --> ");
        Serial.println(ble->mqtt_client_connection);
    }

    else
    {
        ble->mqtt_client_connection = 2;
        Serial.print("mqtt_client_connection_ok --> ");
        Serial.println(ble->mqtt_client_connection);
    }

    Serial.print("mqtt_client_connection_geral --> ");
    Serial.println(ble->mqtt_client_connection);
}
