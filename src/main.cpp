#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "hardware_defs.h"
#include "can_defs.h"
#include "mcp2515_can.h"

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

// Tasks
void TaskSave(void *pvParameters);
void TaskCANFilter(void *pvParameters);

void setup()
{
  // CAN Queue creation
  xQueueCreate(MSG_QUEUE_LEN,sizeof());

  // Setup functions
  pinConfig();           // Hardware and Interrupt Config
  taskSetup();           // Tasks
  setupVolatilePacket(); // volatile packet default values
}

void loop() {}

/* Setup Descriptions */

void taskSetup()
{
  xTaskCreate(TaskCANFilter, "CAN_task", 512, NULL, 5, NULL); // CAN interruption
  xTaskCreate(TaskSave, "SD_task", 512, NULL, 4, NULL);       // SD ticker
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
  volatile_packet.imu.acc_x = 0;
  volatile_packet.imu.acc_y = 0;
  volatile_packet.imu.acc_z = 0;
  volatile_packet.imu.dps_x = 0;
  volatile_packet.imu.dps_y = 0;
  volatile_packet.imu.dps_z = 0;
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
  dataString += String(volatile_packet.imu.acc_x);
  dataString += ",";
  dataString += String(volatile_packet.imu.acc_y);
  dataString += ",";
  dataString += String(volatile_packet.imu.acc_z);
  dataString += ",";
  dataString += String(volatile_packet.imu.dps_x);
  dataString += ",";
  dataString += String(volatile_packet.imu.dps_y);
  dataString += ",";
  dataString += String(volatile_packet.imu.dps_z);

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

void TaskSaving()
{
  sdConfig();

  while (1)
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
      /* If this chunk of code needs to be tested, change
        the saving preiod to something humanly perceptible
        inside include/sh.h
      */
    }

    vTaskDelay(SAVING_PERIOD / portTICK_PERIOD_MS); // Saving frequency
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
  noInterrupts();

  interrupts();
}

void TaskCANFilter(void *pvParameters)
{
  while (1)
  {
    if (xQueueReceive(msg_queue, (void *)&item, 0) == pdTRUE) {

    }
      
  }

  vTaskDelay(5 / portTICK_PERIOD_MS);

  vTaskDelete(NULL);
}