#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "hardware_defs.h"

// #define EMBEDDED

#ifndef EMBEDDED

// vars do timer millis que determina o intervalo entre medidas
unsigned long period = 5; // modifique aqui o intervalo entre medidas, valor em ms
unsigned long time_now = 0;
int pulse_counter = 0;
int num_files = 0;

// Function declarations
int countFiles(File dir);
void pinConfig();
void taskSetup();
void sdConfig();


// Tasks
void TaskSaving(void *pvParameters);

void setup()
{
  // Setup functions
  pinConfig();

  taskSetup();

  taskSetup(); // Tasks
}

void loop()
{

  if (millis() >= time_now + period)
  {
    time_now += period;

    String dataString = "";
    dataString += String(time_now);
    dataString += ",";
    dataString += String(pulse_counter);

    File dataFile = SD.open(FILE_NAME, FILE_WRITE);

    if (dataFile)
    {
      dataFile.println(dataString);
      dataFile.close();
    }

    else
    {
      Serial.println(F("ERRO"));
    }
    pulse_counter = 0;
  }
}

#endif

void taskSetup()
{
  xTaskCreate(TaskSaving, "SD_task", 512, NULL, 5, NULL); // SD ticker
}

void TaskSaving()
{

  sdConfig();

  while (1)
  {
    vTaskDelay(200/portTICK_PERIOD_MS); // Saving frequency
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