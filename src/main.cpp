#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include "hardware_defs.h"
#include "can_defs.h"
#include "mcp2515_can.h"
#include "software_definitions.h"
#include "saving.h"
#include "gprs_defs.h"

HardwareSerial neogps(1);
TinyGPSPlus gps;

// GPRS credentials
const char apn[] = "claro.com.br"; // APN
const char gprsUser[] = "claro";   // User
const char gprsPass[] = "claro";   // Password
const char simPIN[] = "";          // SIM card PIN code, if any
const char *server = "64.227.19.172";
char msg[MSG_BUFFER_SIZE];
char payload_char[MSG_BUFFER_SIZE];

// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 1000;

// ESP hotspot definitions
const char *host = "esp32";                 // Here's your "host device name"
const char *ESP_ssid = "Mangue_Baja_DEV";       // Here's your ESP32 WIFI ssid
const char *ESP_password = "aratucampeaodev"; // Here's your ESP32 WIFI pass

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
void gsmReconnect();
void publishPacket();
void gsmCallback(char *topic, byte *payload, unsigned int length);

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

  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, GPSRX, GPSRX);


}

void loop() {}

/* Setup Descriptions */

void taskSetup()
{
  xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 10000, NULL, 5, NULL, 0);
  // This state machine is responsible for the Basic CAN logging
  xTaskCreatePinnedToCore(ConnStateMachine, "ConnectivityStateMachine", 10000, NULL, 5, NULL, 1);
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
      attachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT), can_ISR, FALLING);
      break;

    default:
      break;
    }
      vTaskDelay(1);

  }
}

void sdConfig()
{
  if (!SD.begin(SD_CS))
  {
    digitalWrite(EMBEDDED_LED, HIGH);
    while (1){
        Serial.printf("vapo");

    }
      
  }
  root = SD.open("/");
  int num_files = countFiles(root);
  sprintf(file_name, "/%s%d.csv", "data", num_files + 1);

  Serial.printf("func");
}

void pinConfig()
{
  // Pins
  pinMode(EMBEDDED_LED, OUTPUT);
  pinMode(CAN_INTERRUPT, INPUT_PULLUP);
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH);

  // Interruptions
  attachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT), can_ISR, FALLING);
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
  dataString += String(volatile_packet.timestamp);
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

void IRAM_ATTR can_ISR()
{
  detachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT));
  l_state = CAN_STATE;
}

void canFilter()
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
      if (messageId == CVT_ID)
      {
        memcpy(&volatile_packet.cvt,(uint16_t*) messageData, len);
      }
      if (messageId == LAT_ID)
      {
        memcpy(&volatile_packet.latitude,(uint16_t*) messageData, len);
      }
      if (messageId == LNG_ID)
      {
        memcpy(&volatile_packet.longitude,(uint16_t*) messageData, len);
      }
      if (messageId == SOC_ID)
      {
        memcpy(&volatile_packet.soc,(uint8_t*) messageData, len);
      }
      if (messageId == VOLT_ID)
      {
        memcpy(&volatile_packet.volt,(uint8_t*) messageData, len);
      }
    }
  }

void ConnStateMachine(void *pvParameters)
{
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.restart();
  // Or, use modem.init() if you don't need the complete restart

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3)
  {
    modem.simUnlock(simPIN);
  }

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork(240000L))
  {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" OK");

  if (modem.isNetworkConnected())
  {
    Serial.println("Network connected");
  }

  Serial.print(F("Connecting to APN: "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" OK");

  // Wi-Fi Config and Debug
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP(ESP_ssid, ESP_password);

  if (!MDNS.begin(host)) // Use MDNS to solve DNS
  {
    // http://esp32.local
    Serial.println("Error configuring mDNS. Rebooting in 1s...");
    delay(1000);
    ESP.restart();
  }
  Serial.println("mDNS configured;");

  mqttClient.setServer(server, PORT);
  mqttClient.setCallback(gsmCallback);

  Serial.println("Ready");
  Serial.print("SoftAP IP address: ");
  Serial.println(WiFi.softAPIP());

    while(1)
    {
      if (!mqttClient.connected())
      {
        gsmReconnect();
      }

      publishPacket();

      for (unsigned long start = millis(); millis() - start < timeoutTime;)
      {
        while (neogps.available())
        {
          if (gps.encode(neogps.read()))
          {
            volatile_packet.latitude = gps.location.lat();
            volatile_packet.longitude = gps.location.lng();
          }
        }
      }

      mqttClient.loop();
      vTaskDelay(1);
    }
  }

/* GPRS Functions */

void gsmCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  memset(payload_char, 0, sizeof(payload_char));

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    payload_char[i] = (char)payload[i];
  }
  Serial.println();
}


void gsmReconnect()
{
  int count = 0;
  //Serial.println("Conecting to MQTT Broker...");
  while (!mqttClient.connected() && count < 3)
  {
    count++;
    //Serial.println("Reconecting to MQTT Broker..");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), "manguebaja", "aratucampeao", "/esp-connected", 2, true, "Offline", true))
    {
      sprintf(msg, "%s", "Online");
      mqttClient.publish("/esp-connected", msg);
      memset(msg, 0, sizeof(msg));
      //Serial.println("Connected.");
      
      /* Subscribe to topics */
      mqttClient.subscribe("/esp-test");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      //Serial.print("Failed with state");
      //Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void publishPacket()
{
  StaticJsonDocument<300> doc;

  doc["accx"] = volatile_packet.imu_acc.acc_x;
  doc["accy"] = volatile_packet.imu_acc.acc_y;
  doc["accz"] = volatile_packet.imu_acc.acc_z;
  doc["rpm"] = volatile_packet.rpm;
  doc["speed"] = volatile_packet.speed;
  doc["motor"] = volatile_packet.temperature;
  doc["flags"] = volatile_packet.flags;
  doc["soc"] = volatile_packet.soc;
  doc["cvt"] = volatile_packet.cvt;
  doc["volt"] = volatile_packet.volt;
  doc["latitude"] = volatile_packet.latitude;
  doc["longitude"] = volatile_packet.longitude;
  doc["timestamp"] = volatile_packet.timestamp;

  memset(msg, 0, sizeof(msg));
  serializeJson(doc, msg);
  mqttClient.publish("/logging", msg);
}
