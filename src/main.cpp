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
#include <Ticker.h>

HardwareSerial neogps(1);
TinyGPSPlus gps;

// SD var
Ticker sdTicker;
Ticker telemetry_state;
bool savingBlink = false;
bool mounted = false; // SD mounted flag

// GPRS credentials
const char apn[] = "timbrasil.br";    // Your APN
const char gprsUser[] = "tim";         // User
const char gprsPass[] = "tim";         // Password
const char simPIN[] = "1010";          // SIM card PIN code, if any

/*Configuração padrão da datelo*/
/*const char apn[] = "datelo.nlt.br";    // Your APN
const char gprsUser[] = "nlt";         // User
const char gprsPass[] = "nlt";         // Password
const char simPIN[] = "6214";          // SIM card PIN code, if any*/

const char *server = "64.227.19.172";
char msg[MSG_BUFFER_SIZE];
char payload_char[MSG_BUFFER_SIZE];

// Define timeout time in milliseconds,0 (example: 2000ms = 2s)
const long timeoutTime = 1000;
boolean flagCANInit = false;

// ESP hotspot defini  tions
const char *host = "esp32";                   // Here's your "host device name"
const char *ESP_ssid = "Mangue_Baja_DEV";     // Here's your ESP32 WIFI ssid
const char *ESP_password = "aratucampeaodev"; // Here's your ESP32 WIFI pass

// vars do timer millis que determina o intervalo entre medidas
int pulse_counter = 0;
int num_files = 0;
bool mode = false;
bool saveFlag = false;
bool gps_send=false;

// Can flag
bool canReady = false;
bool dbgLed = true;

// Function declarations
void sdCallback();
int countFiles(File dir);
void pinConfig();
void taskSetup();
void sdConfig();
void setupVolatilePacket();
void Telemetry_State();
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

void setup()
{

  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  //neogps.begin(9600, SERIAL_8N1, GPSRX, GPSRX);
  
  pinConfig(); // Hardware and Interrupt Config
  
  unsigned long tcanStart = 0, cantimeOut = 0;
  tcanStart = millis();
  cantimeOut = 1000; //(1 segundo)
  // aguarda incializar o shield CAN

  Serial.println("Connecting CAN...");
  while ((millis() - tcanStart) < cantimeOut)
  { // aguarda o timeout
    if (CAN_OK == CAN.begin(CAN_1000KBPS, MCP_8MHz))
    {
      Serial.println("CAN init ok!!!");
      flagCANInit = true; // marca a flag q indica q inicialização correta da CAN
      break; // sai do laço
    }
    flagCANInit = false; // marca a flag q indica q houve problema na inicialização da CAN
  }

  // se houve erro na CAN mostra
  if (!flagCANInit)
  {
    Serial.println("CAN error!!!");
    esp_restart();
  }

  setupVolatilePacket(); // volatile packet default values
  taskSetup();           // Tasks
  sdTicker.attach(1, sdCallback);
  telemetry_state.attach(2, Telemetry_State);
}

void loop() {}

/* Setup Descriptions */
void pinConfig()
{
  // Pins
  pinMode(EMBEDDED_LED, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  pinMode(CAN_INTERRUPT, INPUT_PULLUP);
  // pinMode(MODEM_RST, OUTPUT);
  // digitalWrite(MODEM_RST, HIGH);
  return;
}

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
  volatile_packet.cvt = 0;
  volatile_packet.fuel = 0;
  volatile_packet.volt = 0;
  volatile_packet.latitude = 0;
  volatile_packet.longitude = 0;
  volatile_packet.flags = 0;
  volatile_packet.SOT = 0; /* false */
  volatile_packet.timestamp = 0;
}

void taskSetup()
{
  xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 10000, NULL, 5, NULL, 0);
  // This state machine is responsible for the Basic CAN logging
  xTaskCreatePinnedToCore(ConnStateMachine, "ConnectivityStateMachine", 10000, NULL, 5, NULL, 1);
  // This state machine is responsible for the GPRS, GPS and possible bluetooth connection
}

/* SD State Machine */
void SdStateMachine(void *pvParameters)
{
  while (1)
  { 
    if(saveFlag){
      sdConfig();
      saveFlag = false;
    }
    canFilter();
    vTaskDelay(1);
  }
}

/* SD functions */
void sdConfig()
{
  if (!mounted)
  {
    if (!SD.begin(SD_CS))
    {
      return;
    }

    root = SD.open("/");
    int num_files = countFiles(root);
    sprintf(file_name, "/%s%d.csv", "data", num_files + 1);
    mounted = true;
  }
  sdSave();
}

void sdSave()
{
  dataFile = SD.open(file_name, FILE_APPEND);

  if (dataFile)
  {
    dataFile.println(packetToString());
    dataFile.close();
    savingBlink = !savingBlink;
    digitalWrite(EMBEDDED_LED, savingBlink);

  }

  else
  {
    digitalWrite(EMBEDDED_LED, HIGH);
    Serial.println("falha no save");
  }
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
    dataString += String(volatile_packet.soc);
    dataString += ",";
    dataString += String(volatile_packet.cvt);
    dataString += ",";
    dataString += String(volatile_packet.fuel);
    dataString += ",";
    dataString += String(volatile_packet.volt);
    dataString += ",";
    dataString += String(volatile_packet.flags);
    dataString += ",";
    dataString += String(volatile_packet.timestamp);
    dataString += ",";

  return dataString;
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

void sdCallback() 
{
  saveFlag = true;
}

/* Can functions */

void IRAM_ATTR can_ISR()
{
  detachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT));
  l_state = CAN_STATE;
}

void canFilter()
{

  mode = !mode;
  digitalWrite(DEBUG_LED, mode);
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
      mempcpy(&volatile_packet.rpm, (uint16_t *)messageData, len);
    }
    if (messageId == SPEED_ID)
    {
      mempcpy(&volatile_packet.speed, (uint16_t *)messageData, len);
    }
    if (messageId == TEMPERATURE_ID)
    {
      mempcpy(&volatile_packet.temperature, (uint8_t *)messageData, len);
    }
    if (messageId == FLAGS_ID)
    {
      mempcpy(&volatile_packet.flags, (uint8_t *)messageData, len);
    }
    if (messageId == IMU_ACC_ID)
    {
      memcpy(&volatile_packet.imu_acc, (imu_acc_t *)messageData, len);
    }
    if (messageId == IMU_DPS_ID)
    {
      memcpy(&volatile_packet.imu_dps, (imu_dps_t *)messageData, len);
    }
    if (messageId == CVT_ID)
    {
      memcpy(&volatile_packet.cvt, (uint16_t *)messageData, len);
    }
    if (messageId == LAT_ID)
    {
      memcpy(&volatile_packet.latitude, (uint16_t *)messageData, len);
    }
    if (messageId == LNG_ID)
    {
      memcpy(&volatile_packet.longitude, (uint16_t *)messageData, len);
    }
    if (messageId == SOC_ID)
    {
      memcpy(&volatile_packet.soc, (uint8_t *)messageData, len);
    }
    if (messageId == VOLT_ID)
    {
      memcpy(&volatile_packet.volt, (double *)messageData, len);
    }
    if (messageId == SOT_ID)
    {
      memcpy(&volatile_packet.SOT, (uint8_t *)messageData, len);
    }

    if (messageId == FUEL_ID)
    {
      memcpy(&volatile_packet.fuel, (uint16_t *)messageData, len);
    }
  }
}

/* Telemetry State ticker */
void Telemetry_State()
{
  byte sot[8];
  //byte send_lat[8], send_lng[8];

  sot[0] = volatile_packet.SOT; // 1 byte
  CAN.sendMsgBuf(SOT_ID, false, 8, sot);

  //send_lat[0] = (byte)volatile_packet.latitude; //8 bytes
  //send_lng[0] = (byte)volatile_packet.longitude; //8 bytes

  //CAN.sendMsgBuf(LAT_ID, false, 8, send_lat);
  //CAN.sendMsgBuf(LNG_ID, false, 8, send_lng);
}

/* Connectivity State Machine */
void ConnStateMachine(void *pvParameters)
{
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.restart();
  // Or, use modem.init() if you don't need the complete restart

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  int modemstatus = modem.getSimStatus();
  Serial.print("Status: ");
  Serial.println(modemstatus);

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

  while (1)
  {
    if (!mqttClient.connected())
    {
      volatile_packet.SOT = false;
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
  Serial.println("Conecting to MQTT Broker...");
  while (!mqttClient.connected() && count < 3)
  {
    count++;
    Serial.println("Reconecting to MQTT Broker..");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
      if (mqttClient.connect(clientId.c_str(), "manguebaja", "aratucampeao", "/esp-connected", 2, true, "Offline", true))
      {
        sprintf(msg, "%s", "Online");
        mqttClient.publish("/esp-connected", msg);
        memset(msg, 0, sizeof(msg));
        Serial.println("Connected.");
        volatile_packet.SOT = true;

        /* Subscribe to topics */
        mqttClient.subscribe("/esp-test");
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else {
        Serial.print("Failed with state");
        Serial.println(mqttClient.state());
        volatile_packet.SOT = false;
        delay(2000);
      }
  }
}

void publishPacket()
{
  StaticJsonDocument<300> doc;

  doc["accx"] = (volatile_packet.imu_acc.acc_x*0.061)/1000; 
  doc["accy"] = (volatile_packet.imu_acc.acc_y*0.061)/1000; 
  doc["accz"] = (volatile_packet.imu_acc.acc_z*0.061)/1000; 
  //doc["rpm"] = (volatile_packet.rpm*5000)/65535;
  doc["rpm"] = volatile_packet.rpm;
  doc["speed"] = (volatile_packet.speed*60)/65535; //chega em bytes
  doc["motor"] = volatile_packet.temperature; 
  doc["flags"] = volatile_packet.flags; 
  doc["soc"] = volatile_packet.soc; 
  doc["cvt"] = volatile_packet.cvt; 
  doc["volt"] = volatile_packet.volt;  
  doc["latitude"] = volatile_packet.latitude;
  doc["longitude"] = volatile_packet.longitude;
  doc["Fuel_Level"] = volatile_packet.fuel;
  doc["timestamp"] = volatile_packet.timestamp;

  memset(msg, 0, sizeof(msg));
  serializeJson(doc, msg);
  mqttClient.publish("/logging", msg);
}