#include <Arduino.h>
/* CAN Libraries */
#include "CANmsg.h"
/* OTA functions Librarie */
#include <ota.h>
/* Libraries of SD and Conectivity state Machine */
#include <SD_state_machine.h>
#include <CON_state_machine.h>

TaskHandle_t SDlogging = NULL, ConectivityState = NULL;
bool _sd = false; // flag to check if SD module compile
bool mode = false;
uint8_t _sot = DISCONNECTED;
mqtt_packet_t volatile_packet = setupVolatilePacket(); // volatile packet default values
CANmsg txMsg(CAN_RX_id, CAN_TX_id, CAN_BPS_1000K);

/* CAN functions */
void canISR(CAN_FRAME* rxMsg);
void Send_SOT_msg(void);
/* States Machines */
void SdStateMachine(void* pvParameters);
void ConnStateMachine(void* pvParameters);

void setup()
{
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  
  pinConfig(); // Hardware and Interrupt Config

  /* CAN-BUS Initialize */
  txMsg.Set_Debug_Mode(false);
  txMsg.init(canISR);
  /* if you needed apply a Mask and id to filtering write this:
    * txMsg.init(canISR, ID); or txMsg.init(canISR, ID, Mask);
      * if you ID is bigger then 0x7FF the extended mode is activated
    * you can set the filter in this function too:
    * txMsg.Set_Filter(uint32_t ID, uint32_t Mask, bool Extended); 
  */ 
  
  /* Tasks */
  xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 4096, NULL, 5, &SDlogging, 0);
  // This state machine is responsible for the Basic CAN logging
  xTaskCreatePinnedToCore(ConnStateMachine, "ConnectivityStateMachine", 4096, NULL, 5, &ConectivityState, 1);
  // This state machine is responsible for the GPRS connection          
}

void loop() {/*volatile_packet = update_packet();*/} 

/* SD State Machine */
void SdStateMachine(void* pvParameters)
{
  _sd = start_SD_device();

  /* For synchronization between ECU and panel */
  Send_SOT_msg();

  while(1)
  {
    if(_sd) Check_SD_for_storage(volatile_packet);

    vTaskDelay(1);
  }
}

/* Connectivity State Machine */
void ConnStateMachine(void* pvParameters)
{
  _sot = Initialize_GSM(); 
  if((_sot & 0x04)==ERROR_CONECTION)
  { // enable the error bit
    Send_SOT_msg();
    delay((_sot==0x05 ? DELAY_ERROR : DELAY_ERROR/10));
    esp_restart();
  } 

  Send_SOT_msg();

  while(1)
  {
    if(!Check_mqtt_client_conection())
    {
      _sot==CONNECTED ? _sot ^= (DISCONNECTED|0x01) : 0; // disable online flag 
      Send_SOT_msg();
      _sot = gsmReconnect();
      //while(_sot==DISCONNECTED) { _sot = gsmReconnect(); vTaskDelay(5); }
      Send_SOT_msg(); 
    }

    Send_msg_MQTT(volatile_packet);

    vTaskDelay(1);
  }
}

/* CAN functions */
void canISR(CAN_FRAME* rxMsg)
{ 
  mode = !mode;
  digitalWrite(EMBEDDED_LED, mode);

  volatile_packet.timestamp = millis();

  if(rxMsg->id==IMU_ACC_ID)
  {
    memcpy(&volatile_packet.imu_acc, (imu_acc_t*)&rxMsg->data.uint8, sizeof(imu_acc_t));
    //Serial.printf("ACC X = %f\r\n", (float)((volatile_packet.imu_acc.acc_x*0.061)/1000));
    //Serial.printf("ACC Y = %f\r\n", (float)((volatile_packet.imu_acc.acc_y*0.061)/1000));
    //Serial.printf("ACC Z = %f\r\n", (float)((volatile_packet.imu_acc.acc_z*0.061)/1000));
  }

  if(rxMsg->id==IMU_DPS_ID)
  {
    memcpy(&volatile_packet.imu_dps, (imu_dps_t*)&rxMsg->data.uint8, sizeof(imu_dps_t));
    //Serial.printf("DPS X = %d\r\n", volatile_packet.imu_dps.dps_x);
    //Serial.printf("DPS Y = %d\r\n", volatile_packet.imu_dps.dps_y);
    //Serial.printf("DPS Z = %d\r\n", volatile_packet.imu_dps.dps_z);
  }

  if(rxMsg->id==ANGLE_ID)
  {
    memcpy(&volatile_packet.Angle, (Angle_t*)&rxMsg->data.uint8, sizeof(Angle_t));
    //Serial.printf("Angle Roll = %d\r\n", volatile_packet.Angle.Roll);
    //Serial.printf("Angle Pitch = %d\r\n", volatile_packet.Angle.Pitch);
  }

  if(rxMsg->id==RPM_ID)
  {
    memcpy(&volatile_packet.rpm, (uint16_t*)&rxMsg->data.uint8, sizeof(uint16_t));
    //Serial.printf("RPM = %d\r\n", volatile_packet.rpm);
  }

  if(rxMsg->id==SPEED_ID)
  {
    memcpy(&volatile_packet.speed, (uint16_t*)&rxMsg->data.uint8, sizeof(uint16_t));
    //Serial.printf("Speed = %d\r\n", volatile_packet.speed);
  }

  if(rxMsg->id==TEMPERATURE_ID)
  {
    memcpy(&volatile_packet.temperature, (uint8_t*)&rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("Motor = %d\r\n", volatile_packet.temperature);
  }

  if(rxMsg->id==FLAGS_ID)
  {
    memcpy(&volatile_packet.flags, (uint8_t*)&rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("Flags = %d\r\n", volatile_packet.flags);
  }

  if(rxMsg->id==SOC_ID)
  {
    memcpy(&volatile_packet.SOC, (uint8_t*)&rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("SOC = %d\r\n", volatile_packet.SOC);
  }

  if(rxMsg->id==CVT_ID)
  {
    memcpy(&volatile_packet.cvt, (uint8_t*)&rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("CVT = %d\r\n", volatile_packet.cvt);
  }

  if(rxMsg->id==VOLTAGE_ID)
  {
    memcpy(&volatile_packet.volt, (float*)&rxMsg->data.uint8, sizeof(float));
    //Serial.printf("Volt = %f\r\n", volatile_packet.volt);
  }

  if(rxMsg->id==CURRENT_ID)
  {
    memcpy(&volatile_packet.current, (float*)&rxMsg->data.uint8, sizeof(float));
    //Serial.printf("Current = %f\r\n", volatile_packet.current);
  }

  if(rxMsg->id==LAT_ID)
  {
    memcpy(&volatile_packet.latitude, (double*)&rxMsg->data.uint8, sizeof(double));
    //Serial.printf("Latitude (LAT) = %lf\r\n", volatile_packet.latitude);
  }  

  if(rxMsg->id==LNG_ID)
  {
    memcpy(&volatile_packet.longitude, (double*)&rxMsg->data.uint8, sizeof(double));
    //Serial.printf("Longitude (LNG) = %lf\r\n", volatile_packet.longitude);
  }
}

void Send_SOT_msg()
{
  vTaskDelay(1);
  /* Sent State of Telemetry (SOT) */
  txMsg.clear(SOT_ID);
  txMsg << _sot;
  txMsg.write();

  /*
    * If you send a buffer message you can use this function:
    * SendMsgBuffer(uint32_t Id, bool ext, uint8_t length, unsigned char* data);
    * or you use this:
    * txMsg << data1 << data2 << data3 ...; 
  */
}
