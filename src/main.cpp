#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "hardware_defs.h"
#include "can_defs.h"

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

// Tasks
void TaskSave(void *pvParameters);
void TaskCANFilter(void *pvParameters);

void setup()
{
  // Setup functions
  pinConfig();           // Hardware Config
  taskSetup();           // Tasks
  setupVolatilePacket(); // volatile packet default values
}

void loop(){}

void taskSetup()
{
  xTaskCreate(TaskSave, "SD_task", 512, NULL, 4, NULL);       // SD ticker
  xTaskCreate(TaskCANFilter, "CAN_task", 512, NULL, 5, NULL); // CAN interruption
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
  return dataString;
}

void TaskSaving()
{
  sdConfig();

  while (1)
  {

    File dataFile = SD.open(FILE_NAME, FILE_WRITE);

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

void pulse_counter_ISR()
{
  pulse_counter++;
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
  sprintf(FILE_NAME, "%s%d.csv", "data", num_files + 1);
}

void pinConfig()
{
  pinMode(EMBEDDED_LED, OUTPUT);
  pinMode(CAN_INTERRUPT, INPUT_PULLUP);
  return;
}

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