#include <Arduino.h>
/* CAN Libraries */
#include <CAN.h>
/* Libraries of SD, Conectivity and BLE_DEBUG state Machine */
#include <SD_state_machine.h>
#include <CON_state_machine.h>
#include <BLE_state_machine.h>

/* Task Management */
TaskHandle_t SDlogging = NULL, ConectivityState = NULL, BLE_RESQUEST_State = NULL;

/* SD status variables */
bool _sd = false;  // flag to check if SD module compile
uint8_t sd_status; // flag to check if SD module still working

/* State Of Telemetry (SOT) variables */
uint8_t _sot = DISCONNECTED;

/* Struct for BLE debug */
bluetooth bluetooth_packet;

/* States Machines */
void SdStateMachine(void *pvParameters);
void ConnStateMachine(void *pvParameters);
void BLE_RESQUEST_StateMachine(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  pinConfig(); // Hardware and Interrupt Config

  /* CAN-BUS Initialize */
  if (!CAN_start_device())
    esp_restart();

  memset(&bluetooth_packet, 1, sizeof(bluetooth));

  /* Tasks */
  // This state machine is responsible for the Basic CAN logging
  xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 4096, NULL, 5, &SDlogging, 0);
  // This state machine is responsible for send to the MPU the SCU debug data
  xTaskCreatePinnedToCore(BLE_RESQUEST_StateMachine, "BLE_RESQUEST_StateMachine", 4096, NULL, 3, &BLE_RESQUEST_State, 0);
  // This state machine is responsible for the GPRS connection
  xTaskCreatePinnedToCore(ConnStateMachine, "ConnectivityStateMachine", 4096, NULL, 5, &ConectivityState, 1);
}

void loop() { /**/ }

/* SD State Machine */
void SdStateMachine(void *pvParameters)
{
  _sd = start_SD_device();

  save_SD_data(_sd, &bluetooth_packet);

  while (1)
  {
    bluetooth_packet.check_sd = Check_SD_for_storage();

    save_SD_status_data(&bluetooth_packet);

    vTaskDelay((_sd ? 1 : 100));
  }

  vTaskDelay(1);
}

/* BLE_RESQUEST State Machine */
void BLE_RESQUEST_StateMachine(void *pvParameters)
{
  while (1)
  {
    if (MPU_request_Debug_data())
      Send_SCU_FLAGS(bluetooth_packet);

    vTaskDelay(150 / portTICK_PERIOD_MS);
  }

  vTaskDelay(1);
}

/* Connectivity State Machine */
void ConnStateMachine(void *pvParameters)
{
  Serial.println("Into the conn_function");
  _sot = Initialize_GSM();
  Serial.println("After Initialize_GSM");

  if (_sot == ERROR_CONECTION)
  { // enable the error bit
    Send_SOT_msg(_sot);
    vTaskDelay(DELAY_ERROR(_sot));
  }

  save_SOT_data(_sot, &bluetooth_packet);
  Send_SOT_msg(_sot);

  while (1)
  {
    Serial.print("Into the internet while --> ");
    bool mqtt_client_conection = Check_mqtt_client_conection();
    
    save_mqtt_client_connection_data(mqtt_client_conection, &bluetooth_packet);
    
    if (!mqtt_client_conection)
    {
      _sot == CONNECTED ? _sot = DISCONNECTED : 0; // disable online flag
      Send_SOT_msg(_sot);
      gsmReconnect(_sot);
      Send_SOT_msg(_sot);
    }

    Send_msg_MQTT();

    vTaskDelay(1);
  }

  vTaskDelay(1);
}
