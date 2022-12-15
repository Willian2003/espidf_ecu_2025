#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "hardware_defs.h"
#include "can_defs.h"
#include "mcp2515_can.h"
#include "software_definitions.h"

// vars do timer millis que determina o intervalo entre medidas
int pulse_counter = 0;
int num_files = 0;

// Function declarations
int countFiles(File dir);
void pinConfig();
void taskSetup();
void sdConfig();
void setupVolatilePacket();
String packetToString();
void IRAM_ATTR can_ISR();
void sdSave();
void canFilter();

// State Machines
void SdStateMachine(void *pvParameters);
void ConnStateMachine(void *pvParameters);

// Can flag
boolean canReady = false;

void setup()
{

  timer = millis();

  // Setup functions
  pinConfig();           // Hardware and Interrupt Config
  taskSetup();           // Tasks
  setupVolatilePacket(); // volatile packet default values
}

void loop() {}

/* Setup Descriptions */

void taskSetup()
{
  xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 512, NULL, 5, NULL, 0);
  // This state machine is responsible for the Basic CAN logging
  xTaskCreatePinnedToCore(ConnStateMachine, "ConnectivityStateMachine", 512, NULL, 5, NULL, 1);
  // This state machine is responsible for the GPRS, GPS and possible bluetooth connection

  /* xTaskCreate(TaskCANFilter, "CAN_task", 512, NULL, 5, NULL); // CAN interruption
  xTaskCreate(TaskSave, "SD_task", 512, NULL, 4, NULL);       // SD ticker */
}

void SdStateMachine(void *pvParameters)
{

  sdConfig(); // Opens CSV file
  while (1)
  {
    switch (l_state)
    {
    case IDLE:
      if (millis() - timer > 5)
      {
        l_state = SAVE;
        timer = millis();
      }
      break;

    case SAVE:

      sdSave();

      l_state = IDLE;
      break;

    case CAN_STATE:

      canFilter();

      l_state = IDLE;
      attachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT), canISR, FALLING);
      break;

    default:
      break;
    }
  }
}

void sdConfig()
{
  if (!SD.begin(SD_CS))
  {
    digitalWrite(EMBEDDED_LED, HIGH);
    while (1)
      ;
  }
  root = SD.open("/");
  int num_files = countFiles(root);
  sprintf(file_name, "/%s%d.csv", "data", num_files + 1);
}

void pinConfig()
{
  // Pins
  pinMode(EMBEDDED_LED, OUTPUT);
  pinMode(CAN_INTERRUPT, INPUT_PULLUP);

  // Interruptions
  attachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT), canISR, FALLING);
  return;
}

/* SD functions */

void setupVolatilePacket()
{
  volatile_packet.imu_acc.acc_x = 0;
  volatile_packet.imu_acc.acc_y = 0;
  volatile_packet.imu_acc.acc_z = 0;
  volatile_packet.imu_dps.dps_x = 0;
  volatile_packet.imu_dps.dps_y = 0;
  volatile_packet.imu_dps.dps_z = 0;
  volatile_packet.rpm = 0;
  volatile_packet.speed = 0;
  volatile_packet.temperature = 0;
  volatile_packet.flags = 0;
  volatile_packet.timestamp = 0;
}

String packetToString()
{
  String dataString = "";
  // imu
  dataString += String(volatile_packet.imu_acc.acc_x);
  dataString += ",";
  dataString += String(volatile_packet.imu_acc.acc_y);
  dataString += ",";
  dataString += String(volatile_packet.imu_acc.acc_z);
  dataString += ",";
  dataString += String(volatile_packet.imu_dps.dps_x);
  dataString += ",";
  dataString += String(volatile_packet.imu_dps.dps_y);
  dataString += ",";
  dataString += String(volatile_packet.imu_dps.dps_z);

  dataString += ",";
  dataString += String(volatile_packet.rpm);
  dataString += ",";
  dataString += String(volatile_packet.speed);
  dataString += ",";
  dataString += String(volatile_packet.temperature);
  dataString += ",";
  dataString += String(volatile_packet.flags);
  dataString += ",";
  dataString += String(millis());
  dataString += ",";
  return dataString;
}

void sdSave()
{
  File dataFile = SD.open(file_name, FILE_APPEND);

  if (dataFile)
  {
    dataFile.println(packetToString());
    dataFile.close();
  }

  else
  {
    digitalWrite(EMBEDDED_LED, HIGH);
  }
}

int countFiles(File dir)
{
  int fileCountOnSD = 0; // for counting files
  while (true)
  {

    File entry = dir.openNextFile();
    if (!entry)
    {
      // no more files
      break;
    }
    // for each file count it
    fileCountOnSD++;
    entry.close();
  }
  return fileCountOnSD - 1;
}

/* Can functions */

void IRAM_ATTR canISR()
{
  detachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT));
  l_state = CAN_STATE;
}

void canFilter()
{
  while (1)
  {
    while (CAN_MSGAVAIL == CAN.checkReceive())
    {
      byte messageData[8];
      uint32_t messageId;
      unsigned char len = 0;

      CAN.readMsgBuf(&len, messageData); // Reads message
      messageId = CAN.getCanId();

      volatile_packet.timestamp = millis();

      if (messageId == RPM_ID)
      {
        mempcpy(&volatile_packet.rpm, (uint16_t*)messageData, len);
      }
      if (messageId == SPEED_ID)
      {
        mempcpy(&volatile_packet.speed, (uint16_t*)messageData, len);
      }
      if (messageId == TEMPERATURE_ID)
      {
        mempcpy(&volatile_packet.temperature, (uint8_t*)messageData, len);
      }
      if (messageId == FLAGS_ID)
      {
        mempcpy(&volatile_packet.flags, (uint8_t*)messageData, len);
      }
      if (messageId == IMU_ACC_ID)
      {
        memcpy(&volatile_packet.imu_acc,(imu_acc_t*) messageData, len);
      }
      if (messageId == IMU_DPS_ID)
      {
        memcpy(&volatile_packet.imu_dps,(imu_dps_t*) messageData, len);
      }
    }
  }