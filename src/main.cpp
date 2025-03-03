#include <Arduino.h>
/* CAN Libraries */
#include <CAN.h>
/* Libraries of SD, Conectivity(LTE) and BLE_DEBUG state Machine */
#include <SD_state_machine.h>
#include <CON_state_machine.h>

/* Task Management */
TaskHandle_t SDlogging = NULL, ConectivityState = NULL, BLE_RESQUEST_State = NULL;

/* SD status variables */
uint8_t _sd = FAIL_RESPONSE;       // flag to check if SD module compile
uint8_t sd_status = FAIL_RESPONSE; // flag to check if SD module still working

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

  mqtt_packet_t s;
  uint8_t as[sizeof(mqtt_packet_t)];
  memcpy(as, (uint8_t*)&s, sizeof(mqtt_packet_t));
  Serial.print("[");
  for (int i = 0; i < sizeof(mqtt_packet_t); i++)
  {
    Serial.print(as[i], HEX);
    Serial.print("  ");

  }
  Serial.print("]");

  /* Hardware and Interrupt Config */
  pinMode(EMBEDDED_LED, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);

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
  bluetooth_packet.sd_start = _sd = start_SD_device();
  //Serial.printf("_sd --> %d\r\n", bluetooth_packet.sd_start);

  while (1)
  {
    bluetooth_packet.check_sd = Check_SD_for_storage();

    //Serial.println("Out the function");
    //Serial.printf("bluetooth_packet.check_sd --> %d\r\n", bluetooth_packet.check_sd);

    vTaskDelay(1 / portTICK_PERIOD_MS);
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

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  vTaskDelay(1);
}

/* Connectivity State Machine */
void ConnStateMachine(void *pvParameters)
{
  //Serial.println("Into the conn_function");
  _sot = Initialize_GSM();
  //Serial.println("After Initialize_GSM");

  if (_sot == ERROR_CONECTION)
  { // enable the error bit
    Send_SOT_msg(_sot);
    bluetooth_packet.internet_modem = FAIL_RESPONSE;
    //Serial.printf("internet_modem_erro --> %d\r\n", bluetooth_packet.internet_modem);
    vTaskDelay(DELAY_ERROR(_sot));
  }
  
  else
  {
    bluetooth_packet.internet_modem = SUCESS_RESPONSE;
    //Serial.printf("internet_modem_ok --> %d\r\n", bluetooth_packet.internet_modem);
  }

  //Serial.printf("internet_modem_geral --> %d\r\n", bluetooth_packet.internet_modem);
  Send_SOT_msg(_sot);

  while (1)
  {
    //Serial.print("Into the internet while --> ");  
    
    if (!Check_mqtt_client_conection())
    {
      bluetooth_packet.mqtt_client_connection = FAIL_RESPONSE;
      //Serial.printf("mqtt_client_connection_erro --> %d\r\n", bluetooth_packet.mqtt_client_connection);
      _sot == CONNECTED ? _sot = DISCONNECTED : 0; // disable online flag
      Send_SOT_msg(_sot);
      gsmReconnect(_sot);
      Send_SOT_msg(_sot);
    }

    else
    {
      bluetooth_packet.mqtt_client_connection = SUCESS_RESPONSE;
      //Serial.print("mqtt_client_connection_ok --> %d\r\n", bluetooth_packet.mqtt_client_connection);
    }

    //Serial.printf("mqtt_client_connection_geral --> %d\r\n", bluetooth_packet.mqtt_client_connection);

    Send_msg_MQTT();

    vTaskDelay(1);
  }

  vTaskDelay(1);
}
