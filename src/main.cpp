#include <Arduino.h>
/* ESP Tools Libraries */
#include <Ticker.h>
#include <SD.h>
#include "esp32_can.h"
/* Communication Libraries */
#include "can_defs.h"
#include "gprs_defs.h"
/* User Libraries */
#include "middle_defs.h"
#include "hardware_defs.h"

#define MB1 // Uncomment a line if it is your car choice
//#define MB2 // Uncomment a line if it is your car choice

#ifdef MB1
  #define CAR_ID MB1_ID
#endif

#ifdef MB2
  #define CAR_ID MB2_ID
#endif

/* Credentials Variables */
#define TIM     // Uncomment this line and comment the others if this is your chip
//#define CLARO   // Uncomment this line and comment the others if this is your chip
//#define VIVO    // Uncomment this line and comment the others if this is your chip

/* GPRS credentials */
#ifdef TIM
  const char apn[] = "timbrasil.br";    // Your APN
  const char gprsUser[] = "tim";        // User
  const char gprsPass[] = "tim";        // Password
  const char simPIN[] = "1010";         // SIM card PIN code, if any
#elif defined(CLARO)
  const char apn[] = "claro.com.br";    // Your APN
  const char gprsUser[] = "claro";      // User
  const char gprsPass[] = "claro";      // Password
  const char simPIN[] = "3636";         // SIM cad PIN code, id any
#elif defined(VIVO)
  const char apn[] = "zap.vivo.com.br";  // Your APN
  const char gprsUser[] = "vivo";        // User
  const char gprsPass[] = "vivo";        // Password
  const char simPIN[] = "8486";          // SIM cad PIN code, id any
#else
  const char apn[] = "timbrasil.br";    // Your APN
  const char gprsUser[] = "tim";        // User
  const char gprsPass[] = "tim";        // Password
  const char simPIN[] = "1010";         // SIM card PIN code, if any
#endif

/* ESP Tools */
CAN_FRAME txMsg;
Ticker timeoutSOT; 
Ticker ticker40Hz;

/* Debug Variables */
bool savingBlink = false;
bool mounted = false;
/* Global Variables */
uint8_t SOT = DISCONNECTED;
const char *server = "64.227.19.172";
char msg[MSG_BUFFER_SIZE];
char payload_char[MSG_BUFFER_SIZE];

// Define timeout time in milliseconds,0 (example: 2000ms = 2s)
const long timeoutTime = 3000;

// ESP hotspot definitions
const char *host = "esp32";                   // Here's your "host device name"
const char *ESP_ssid = "Mangue_Baja_DEV";     // Here's your ESP32 WIFI ssid
const char *ESP_password = "aratucampeaodev"; // Here's your ESP32 WIFI pass

// SD variables
char file_name[20];
File root, dataFile;

// vars do timer millis que determina o intervalo entre medidas
int pulse_counter = 0;
bool mode = false;
bool saveFlag = false;

/* States Machines */
void SdStateMachine(void *pvParameters);
void ConnStateMachine(void *pvParameters);
/* Interrupts routine */
void canISR(CAN_FRAME *rxMsg);
void debouceHandlerSOT();
void ticker40HzISR();
/* Setup Descriptions */
void pinConfig();    
void setupVolatilePacket();
void taskSetup(); 
/* SD State Machine Global Functions */
bool sdConfig();
int countFiles(File dir);
String packetToString(bool err);
void sdSave(bool set); 
/* Connectivity State Machine Global Functions */
void gsmCallback(char *topic, byte *payload, unsigned int length);
void gsmReconnect();
void publishPacket();

void setup()
{
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_TX, MODEM_RX);
  
  pinConfig(); // Hardware and Interrupt Config

  /* CAN-BUS initialize */
  CAN.setCANPins((gpio_num_t)CAN_RX_id, (gpio_num_t)CAN_TX_id);
  CAN.begin(CAN_BPS_1000K);
  CAN.watchFor();
  CAN.setCallback(0, canISR);
  txMsg.length = 8;
  txMsg.extended = 0; txMsg.rtr = 0;
  txMsg.id = SOT_ID; 

  setupVolatilePacket(); // volatile packet default values
  taskSetup();           // Tasks

  ticker40Hz.attach(0.025, ticker40HzISR);
}

void loop() {/* Dont Write here */} 

/* Setup Descriptions */
void pinConfig()
{
  // Pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT);
  //pinMode(CAN_INTERRUPT, INPUT_PULLUP);
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
  volatile_packet.Angle.Roll    = 0;
  volatile_packet.Angle.Pitch   = 0;
  volatile_packet.rpm           = 0;
  volatile_packet.speed         = 0;
  volatile_packet.temperature   = 0;
  volatile_packet.flags         = 0;
  volatile_packet.SOC           = 0;
  volatile_packet.cvt           = 0;
  //volatile_packet.fuel          = 0;
  volatile_packet.volt          = 0;
  volatile_packet.current       = 0;
  volatile_packet.latitude      = -12.70814; 
  volatile_packet.longitude     = -38.1732; 
  volatile_packet.timestamp     = 0;
}

void taskSetup()
{
  xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 10000, NULL, 5, NULL, 0);
  // This state machine is responsible for the Basic CAN logging
  xTaskCreatePinnedToCore(ConnStateMachine, "ConnectivityStateMachine", 10000, NULL, 5, NULL, 1);
  // This state machine is responsible for the GPRS and possible bluetooth connection
}

/* SD State Machine */
void SdStateMachine(void *pvParameters)
{
  do { Serial.println("Mount SD..."); } while(!sdConfig() && millis() < timeoutTime);

  if(!mounted) 
  { 
    Serial.println("SD mounted error!!"); 
    return; 
  }

  sdSave(true);

  /* For synchronization between ECU and panel */
  timeoutSOT.once(0.1, debouceHandlerSOT);

  while(1)
  {
    if(saveFlag)
    {
      sdSave(false);
      saveFlag = false;
    }

    vTaskDelay(1);
  }
}

/* SD State Machine Global Functions */
bool sdConfig()
{
  if(!SD.begin(SD_CS)) return false;

  root = SD.open("/");
  int num_files = countFiles(root);
  sprintf(file_name, "/%s%d.csv", "data", num_files+1);
  mounted = true;

  return true;
}

int countFiles(File dir)
{
  int fileCountOnSD = 0; // for counting files
  for(;;)
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

void sdSave(bool set)
{
  dataFile = SD.open(file_name, FILE_APPEND);

  if(dataFile)
  {
    dataFile.println(packetToString(set));
    dataFile.close();
    savingBlink = !savingBlink;
    digitalWrite(DEBUG_LED, savingBlink);
  } else {
    digitalWrite(DEBUG_LED, LOW);
    Serial.println(F("falha no save"));
  }
}

String packetToString(bool err)
{
  String dataString = "";
    if(err)
    {
      dataString += "ACCX";
      dataString += ",";
      dataString += "ACCY";
      dataString += ",";
      dataString += "ACCZ";
      dataString += ",";
      dataString += "DPSX";
      dataString += ",";
      dataString += "DPSY";
      dataString += ",";
      dataString += "DPSZ";
      dataString += ",";
      dataString += "ROLL";
      dataString += ",";
      dataString += "PITCH";
      dataString += ",";

      dataString += "RPM";
      dataString += ",";
      dataString += "VEL";
      dataString += ",";
      dataString += "TEMP_MOTOR";
      dataString += ",";
      dataString += "SOC";
      dataString += ",";
      dataString += "TEMP_CVT";
      dataString += ",";
      //dataString += "FUEL_LEVEL";
      //dataString += ",";
      dataString += "VOLT";
      dataString += ",";
      dataString += "CURRENT";
      dataString += ",";
      dataString += "FLAGS";
      dataString += ",";
      dataString += "LATITUDE";
      dataString += ",";
      dataString += "LONGITUDE";
      dataString += ",";
      dataString += "TIMESTAMP";
      dataString += ",";
      dataString += "ID=" + String(CAR_ID);
    }
    
    else
    {
      // imu
      dataString += String((volatile_packet.imu_acc.acc_x*0.061)/1000);
      dataString += ",";
      dataString += String((volatile_packet.imu_acc.acc_y*0.061)/1000);
      dataString += ",";
      dataString += String((volatile_packet.imu_acc.acc_z*0.061)/1000);
      dataString += ",";
      dataString += String(volatile_packet.imu_dps.dps_x);
      dataString += ",";
      dataString += String(volatile_packet.imu_dps.dps_y);
      dataString += ",";
      dataString += String(volatile_packet.imu_dps.dps_z);
      dataString += ",";
      dataString += String(volatile_packet.Angle.Roll);
      dataString += ",";
      dataString += String(volatile_packet.Angle.Pitch);

      dataString += ",";
      dataString += String(volatile_packet.rpm);
      dataString += ",";
      dataString += String(volatile_packet.speed);
      dataString += ",";
      dataString += String(volatile_packet.temperature);
      dataString += ",";
      dataString += String(volatile_packet.SOC);
      dataString += ",";
      dataString += String(volatile_packet.cvt);
      dataString += ",";
      //dataString += String(volatile_packet.fuel);
      //dataString += ",";
      dataString += String(volatile_packet.volt);
      dataString += ",";
      dataString += String(volatile_packet.current);
      dataString += ",";
      dataString += String(volatile_packet.flags);
      dataString += ",";
      dataString += String(volatile_packet.latitude);
      dataString += ",";
      dataString += String(volatile_packet.longitude);
      dataString += ",";
      dataString += String(volatile_packet.timestamp);
    }

  return dataString;
}

/* Connectivity State Machine */
void ConnStateMachine(void *pvParameters)
{
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.restart();
  // Or, use modem.init() if you don't need the complete restart

  Serial.print("Modem: "); Serial.println(modem.getModemInfo());

  Serial.print("Status: "); Serial.println(modem.getSimStatus());

  // Unlock your SIM card with a PIN if needed
  if(strlen(simPIN) && modem.getSimStatus() != 3)
  {
    modem.simUnlock(simPIN);
  }

  Serial.print("Waiting for network...");
  if(!modem.waitForNetwork(240000L))
  {
    Serial.println("fail");
    SOT |= ERROR_CONECTION;
    timeoutSOT.once(0.1, debouceHandlerSOT);
    delay(10000);
    return;
  }
  Serial.println("OK");

  if(modem.isNetworkConnected())
  {
    Serial.println("Network connected");
  }

  Serial.print(F("Connecting to APN: "));
  Serial.print(apn);
  if(!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    Serial.println(" fail");
    SOT |= ERROR_CONECTION;
    timeoutSOT.once(0.1, debouceHandlerSOT);
    delay(10000);
    return;
  }
  Serial.println(" OK");

  // Wi-Fi Config and Debug
  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP(ESP_ssid, ESP_password);

  if(!MDNS.begin(host)) // Use MDNS to solve DNS
  {
    // http://esp32.local
    Serial.println("Error configuring mDNS. Rebooting in 1s...");
    SOT |= ERROR_CONECTION;
    timeoutSOT.once(0.1, debouceHandlerSOT);
    delay(1000);
    ESP.restart();
  }
  Serial.println("mDNS configured;");

  mqttClient.setServer(server, PORT);
  mqttClient.setCallback(gsmCallback);

  Serial.println("Ready");
  Serial.print("SoftAP IP address: "); Serial.println(WiFi.softAPIP());

  while(1)
  {
    if(!mqttClient.connected())
    {
      SOT = DISCONNECTED; // disable online flag 
      timeoutSOT.once(0.1, debouceHandlerSOT);
      gsmReconnect();
    }

    publishPacket();

    mqttClient.loop();
    vTaskDelay(1);
  }
}

/* Connectivity State Machine Global Functions */
// GPRS Functions
void gsmCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  memset(payload_char, 0, sizeof(payload_char));

  for(int i=0; i<length; i++)
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
  while(!mqttClient.connected() && count < 3)
  {
    count++;
    Serial.println("Reconecting to MQTT Broker..");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);

    if(mqttClient.connect(clientId.c_str(), "manguebaja", "aratucampeao", "/esp-connected", 2, true, "Offline", true))
    {
      sprintf(msg, "%s", "Online");
      mqttClient.publish("/esp-connected", msg);
      memset(msg, 0, sizeof(msg));
      Serial.println("Connected.");
      SOT |= CONNECTED; // enable online flag 
      timeoutSOT.once(0.1, debouceHandlerSOT);

      /* Subscribe to topics */
      mqttClient.subscribe("/esp-test");
      //digitalWrite(LED_BUILTIN, HIGH);
    } else {
      Serial.print("Failed with state");
      Serial.println(mqttClient.state());
      SOT &= ~(CONNECTED); // disable online flag 
      timeoutSOT.once(0.1, debouceHandlerSOT);
      delay(2000); 
    }
  }
}

void publishPacket()  
{
  StaticJsonDocument<305> doc;

  doc["accx"] = (volatile_packet.imu_acc.acc_x*0.061)/1000;
  doc["accy"] = (volatile_packet.imu_acc.acc_y*0.061)/1000; 
  doc["accz"] = (volatile_packet.imu_acc.acc_z*0.061)/1000; 
  doc["dpsx"] = volatile_packet.imu_dps.dps_x;
  doc["dpsy"] = volatile_packet.imu_dps.dps_y;
  doc["dpsz"] = volatile_packet.imu_dps.dps_z;
  doc["roll"] = volatile_packet.Angle.Roll;
  doc["pitch"] = volatile_packet.Angle.Pitch;
  doc["rpm"] = volatile_packet.rpm;
  doc["speed"] = volatile_packet.speed;
  doc["motor"] = volatile_packet.temperature;
  doc["flags"] = volatile_packet.flags;
  doc["soc"] = volatile_packet.SOC; 
  doc["cvt"] = volatile_packet.cvt; 
  doc["volt"] = volatile_packet.volt; 
  doc["current"] = volatile_packet.current; 
  doc["latitude"] = volatile_packet.latitude;
  doc["longitude"] = volatile_packet.longitude;
  //doc["fuel_level"] = volatile_packet.fuel;
  doc["timestamp"] = volatile_packet.timestamp;

  //Serial.printf("Json Size = %d\r\n", doc.size());

  memset(msg, 0, sizeof(msg));
  serializeJson(doc, msg);
  mqttClient.publish("/logging", msg);
}

/* Interrupts routine */
void canISR(CAN_FRAME *rxMsg)
{
  mode = !mode;
  digitalWrite(EMBEDDED_LED, mode);

  volatile_packet.timestamp = millis();

  if(rxMsg->id==IMU_ACC_ID)
  {
    memcpy(&volatile_packet.imu_acc, (imu_acc_t *)rxMsg->data.uint8, sizeof(imu_acc_t));
    //Serial.printf("ACC Z = %f\r\n", (float)((volatile_packet.imu_acc.acc_z*0.061)/1000));
    //Serial.printf("ACC X = %f\r\n", (float)((volatile_packet.imu_acc.acc_x*0.061)/1000));
    //Serial.printf("ACC Y = %f\r\n", (float)((volatile_packet.imu_acc.acc_y*0.061)/1000));
  }

  if(rxMsg->id==IMU_DPS_ID)
  {
    memcpy(&volatile_packet.imu_dps, (imu_dps_t *)rxMsg->data.uint8, sizeof(imu_dps_t));
    //Serial.printf("DPS X = %d\r\n", volatile_packet.imu_dps.dps_x);
    //Serial.printf("DPS Y = %d\r\n", volatile_packet.imu_dps.dps_y);
    //Serial.printf("DPS Z = %d\r\n", volatile_packet.imu_dps.dps_z);
  }

  if(rxMsg->id==ANGLE_ID)
  {
    memcpy(&volatile_packet.Angle, (Angle_t *)rxMsg->data.uint8, sizeof(Angle_t));
    //Serial.printf("Angle Roll = %d\r\n", volatile_packet.Angle.Roll);
    //Serial.printf("Angle Pitch = %d\r\n", volatile_packet.Angle.Pitch);
  }

  if(rxMsg->id==RPM_ID)
  {
    memcpy(&volatile_packet.rpm, (uint16_t *)rxMsg->data.uint8, sizeof(uint16_t));
    //Serial.printf("RPM = %d\r\n", volatile_packet.rpm);
  }

  if(rxMsg->id==SPEED_ID)
  {
    memcpy(&volatile_packet.speed, (uint16_t *)rxMsg->data.uint8, sizeof(uint16_t));
    //Serial.printf("Speed = %d\r\n", volatile_packet.speed);
  }

  if(rxMsg->id==TEMPERATURE_ID)
  {
    memcpy(&volatile_packet.temperature, (uint8_t *)rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("Motor = %d\r\n", volatile_packet.temperature);
  }

  if(rxMsg->id==FLAGS_ID)
  {
    memcpy(&volatile_packet.flags, (uint8_t *)rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("Flags = %d\r\n", volatile_packet.flags);
  }

  if(rxMsg->id==SOC_ID)
  {
    memcpy(&volatile_packet.SOC, (uint8_t *)rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("SOC = %d\r\n", volatile_packet.SOC);
  }

  if(rxMsg->id==CVT_ID)
  {
    memcpy(&volatile_packet.cvt, (uint8_t *)rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("CVT = %d\r\n", volatile_packet.cvt);
  }

  if(rxMsg->id==VOLTAGE_ID)
  {
    memcpy(&volatile_packet.volt, (float *)rxMsg->data.uint8, sizeof(float));
    //Serial.printf("Volt = %f\r\n", volatile_packet.volt);
  }

  if(rxMsg->id==CURRENT_ID)
  {
    memcpy(&volatile_packet.current, (float *)rxMsg->data.uint8, sizeof(float));
    //Serial.printf("Current = %f\r\n", volatile_packet.current);
  }

  if(rxMsg->id==LAT_ID)
  {
    memcpy(&volatile_packet.latitude, (double *)rxMsg->data.uint8, sizeof(double));
    //Serial.printf("Latitude (LAT) = %lf\r\n", volatile_packet.latitude);
  }  

  if(rxMsg->id==LNG_ID)
  {
    memcpy(&volatile_packet.longitude, (double *)rxMsg->data.uint8, sizeof(double));
    //Serial.printf("Longitude (LNG) = %lf\r\n", volatile_packet.longitude);
  }
}

void debouceHandlerSOT()
{
  /* Sent State of Telemetry (SOT) */
  txMsg.data.uint8[0] = SOT;
  CAN.sendFrame(txMsg); 
}

void ticker40HzISR()
{
  saveFlag = true;
}
