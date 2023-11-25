#include <Arduino.h>
#include <Ticker.h>
#include <SD.h>
#include <CircularBuffer.h>
#include <mcp2515_can.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include "can_defs.h"
#include "middle_defs.h"
#include "hardware_defs.h"
#include "gprs_defs.h"
#include "saving.h"

/* Credentials Variables */
//#define TIM     // Uncomment this line and comment the others if this is your chip
//#define CLARO   // Uncomment this line and comment the others if this is your chip
#define VIVO    // Uncomment this line and comment the others if this is your chip

// GPRS credentials
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
#endif

/* General Libraries */
//HardwareSerial neogps(1);
//TinyGPSPlus gps;

/* ESP Tools */
CircularBuffer<state_t, BUFFER_SIZE/2> state_buffer;
state_t current_state = IDLE_ST;
Ticker ticker1Hz; 

/* Debug Variables */
float Debug_accx = 0.0;
bool buffer_full = false;
bool mounted = false; // SD mounted flag
bool savingBlink = false;
/* Global Variables */
const char *server = "64.227.19.172";
char msg[MSG_BUFFER_SIZE];
char payload_char[MSG_BUFFER_SIZE];

// Define timeout time in milliseconds,0 (example: 2000ms = 2s)
const long timeoutTime = 1000;
boolean flagCANInit = false;

// ESP hotspot definitions
const char *host = "esp32";                   // Here's your "host device name"
const char *ESP_ssid = "Mangue_Baja_DEV";     // Here's your ESP32 WIFI ssid
const char *ESP_password = "aratucampeaodev"; // Here's your ESP32 WIFI pass

// vars do timer millis que determina o intervalo entre medidas
int pulse_counter = 0;
int num_files = 0;
bool mode = false;
bool saveFlag = false;

/* States Machines */
void SdStateMachine(void *pvParameters);
void ConnStateMachine(void *pvParameters);
/* Interrupts routine */
void IRAM_ATTR can_ISR();
void ticker1HzISR();
/* Setup Descriptions */
void pinConfig();    
void setupVolatilePacket();
void taskSetup(); 
/* SD State Machine Global Functions */         
// CAN transmitter function
void RingBuffer_state();
// SD Functions
void sdConfig();
void sdSave();
String packetToString();
int countFiles(File dir);
// CAN receiver function
void canFilter();
/* Connectivity State Machine Global Functions */
// GPRS Functions
void gsmCallback(char *topic, byte *payload, unsigned int length);
void gsmReconnect();
void publishPacket();

void setup()
{
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  //neogps.begin(9600, SERIAL_8N1, GPSRX, GPSRX);
  
  pinConfig(); // Hardware and Interrupt Config
  
  unsigned long tcanStart = 0, cantimeOut = 0;
  tcanStart = millis();
  cantimeOut = 1000; // (1 second)
  // wait for the CAN shield to initialize

  Serial.println("Connecting CAN...");
  while((millis() - tcanStart) < cantimeOut) // wait timeout
  { 
    if(CAN.begin(CAN_1000KBPS, MCP_8MHz) == CAN_OK)
    {
      Serial.println("CAN init ok!!!");
      flagCANInit = true; // Marks the flag that indicates correct CAN initialization
      break; // get out of the loop
    }
    flagCANInit = false; // Mark the flag that indicates there was a problem initializing the CAN
  }

  // if there was an error in the CAN it shows
  if(!flagCANInit)
  {
    Serial.println("CAN error!!!");
    esp_restart();
  }

  setupVolatilePacket(); // volatile packet default values
  taskSetup();           // Tasks

  ticker1Hz.attach(1, ticker1HzISR);
}

void loop() {/* Dont Write here */} 

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
  volatile_packet.Angle.Roll = 0;
  volatile_packet.Angle.Pitch = 0;
  volatile_packet.rpm = 0;
  volatile_packet.speed = 0;
  volatile_packet.temperature = 0;
  volatile_packet.flags = 0;
  volatile_packet.SOC = 0;
  volatile_packet.cvt = 0;
  volatile_packet.fuel = 0;
  volatile_packet.volt = 0;
  volatile_packet.current = 0;
  volatile_packet.latitude = -12.70814; 
  volatile_packet.longitude = -38.1732; 
  volatile_packet.timestamp = 0;
  volatile_packet.SOT = 0;
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
  while(1)
  {
    RingBuffer_state(); 
    if(saveFlag)
    {
      sdConfig();
      saveFlag = false;
    }
    canFilter();
    vTaskDelay(1);
  }
}

/* SD State Machine Global Functions */
// CAN transmitter function
void RingBuffer_state()
{
  if(state_buffer.isFull())
  {
    buffer_full=true;
    current_state = state_buffer.pop();
  } else {
    buffer_full=false;
    if(!state_buffer.isEmpty())
      current_state = state_buffer.pop();
    else
      current_state = IDLE_ST;
  }

  switch(current_state)
  {
    case IDLE_ST:
      //Serial.println("i");
      break;
    
    case SOT_ST:
      //Serial.println("sot");
      byte sot[8];
      sot[0] = volatile_packet.SOT; // 1 byte

      /* Send State of Telemetry message */
      if(CAN.sendMsgBuf(SOT_ID, false, 8, sot)==CAN_OK)
      {
        //Serial.printf("\r\nSOT = %d\r\n", volatile_packet.SOT);
      }

      break;

    case GPS_ST:
      //Serial.println("gps");
      byte send_lat[8], send_lng[8];
      send_lat[0] = volatile_packet.latitude;  // 8 bytes
      send_lng[0] = volatile_packet.longitude; // 8 bytes

      /* Send latitude message */
      if(CAN.sendMsgBuf(LAT_ID, false, 8, send_lat)==CAN_OK) 
      {
        //Send the longitude message only the latitude has successful 
        //Serial.printf("\r\nLatitude = %lf\r\n", volatile_packet.latitude);
        CAN.sendMsgBuf(LNG_ID, false, 8, send_lng);
      }

      break;

    case DEBUG_ST:
      //Serial.println("d");
      //Serial.printf("\r\nSOT = %d\r\n", volatile_packet.SOT);
      //Serial.printf("\r\nLatitude = %lf\r\n", volatile_packet.latitude);
      //Serial.printf("\r\nLongitude = %lf\r\n", volatile_packet.longitude);
      break;
  }
}

// SD Functions
void sdConfig()
{
  if (!mounted)
  {
    if(!SD.begin(SD_CS)) { return; } 

    root = SD.open("/");
    int num_files = countFiles(root);
    sprintf(file_name, "/%s%d.csv", "data", num_files + 1);

    dataFile = SD.open(file_name, FILE_APPEND);

    if(dataFile)
    {
      dataFile.println(packetToString());
      dataFile.close();
    } else {
      digitalWrite(DEBUG_LED, HIGH);
      Serial.println(F("FAIL TO OPEN THE FILE"));
    }
    mounted = true;
  }
  sdSave();
}

int countFiles(File dir)
{
  int fileCountOnSD = 0; // for counting files
  while(true)
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

void sdSave()
{
  dataFile = SD.open(file_name, FILE_APPEND);

  if(dataFile)
  {
    dataFile.println(packetToString());
    dataFile.close();
    savingBlink = !savingBlink;
    digitalWrite(DEBUG_LED, savingBlink);
  } else {
    digitalWrite(DEBUG_LED, LOW);
    Serial.println(F("falha no save"));
  }
}

String packetToString()
{
  String dataString = "";
    if(!mounted)
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
      dataString += "FUEL_LEVEL";
      dataString += ",";
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
    }
    
    else
    {
      // imu
      //dataString += String((volatile_packet.imu_acc.acc_x*0.061)/1000);
      dataString += String(Debug_accx);
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
      dataString += String(volatile_packet.fuel);
      dataString += ",";
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

// CAN receiver function
void canFilter()
{
  //mode = !mode;
  //digitalWrite(EMBEDDED_LED, mode);
  while(CAN.checkReceive() == CAN_MSGAVAIL)
  {
    //Serial.println("ok!");
    mode = !mode;
    digitalWrite(EMBEDDED_LED, mode);

    byte messageData[8];
    uint32_t messageId;
    unsigned char len = 0;

    // Reads message and ID
    CAN.readMsgBuf(&len, messageData); 
    messageId = CAN.getCanId();

    /* Debug DATA */
    volatile_packet.timestamp = millis();

    /* Battery management DATA */
    if(messageId == VOLTAGE_ID)
    {
      memcpy(&volatile_packet.volt, (float *)messageData, len); 
      //Serial.printf("\r\nVoltage = %f\r\n", volatile_packet.volt);
    }

    if(messageId == SOC_ID)
    {
      memcpy(&volatile_packet.SOC, (uint8_t *)messageData, len);
      //Serial.printf("\r\nState Of Charge = %d\r\n", volatile_packet.SOC);
    }

    if(messageId == CURRENT_ID)
    {
      memcpy(&volatile_packet.current, (float *)messageData, len);
      //Serial.printf("\r\nCurrent = %f\r\n", volatile_packet.current);
    }

    /* Rear DATA */
    if(messageId == CVT_ID) // Old BMU
     {
      memcpy(&volatile_packet.cvt, (uint8_t *)messageData, len);
      //Serial.printf("\r\nCVT temperature = %d\r\n", volatile_packet.cvt);
    }

    if(messageId == FUEL_ID) // Old BMU
    {
      memcpy(&volatile_packet.fuel, (uint16_t *)messageData, len);
      //Serial.printf("\r\nFuel Level = %d\r\n", volatile_packet.fuel);
    }

    if(messageId == TEMPERATURE_ID)
    {
      mempcpy(&volatile_packet.temperature, (uint8_t *)messageData, len);
      //Serial.printf("\r\nMotor temperature = %d\r\n", volatile_packet.temperature);
    } 

    if(messageId == FLAGS_ID)
    {
      mempcpy(&volatile_packet.flags, (uint8_t *)messageData, len);
      //Serial.printf("\r\nflags = %d\r\n", volatile_packet.flags);
    }

    if(messageId == RPM_ID)
    {
      mempcpy(&volatile_packet.rpm, (uint16_t *)messageData, len);
      //Serial.printf("\r\nRPM = %d\r\n", volatile_packet.rpm);
    }
    
    /* Front DATA */
    if(messageId == SPEED_ID)
    {
      mempcpy(&volatile_packet.speed, (uint16_t *)messageData, len);
      //Serial.printf("\r\nSpeed = %d\r\n", volatile_packet.speed);
    }  

    if(messageId == IMU_ACC_ID)
    {
      memcpy(&volatile_packet.imu_acc, (imu_acc_t *)messageData, len);
      Debug_accx = ((float)volatile_packet.imu_acc.acc_x*0.061)/1000.00;
      //Serial.printf("\r\nAccx = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_x*0.061)/1000));
      //Serial.printf("\r\nAccy = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_y*0.061)/1000));
      //Serial.printf("\r\nAccz = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_z*0.061)/1000));
    }

    if(messageId == IMU_DPS_ID)
    {
      memcpy(&volatile_packet.imu_dps, (imu_dps_t *)messageData, len);
      //Serial.printf("\r\nDPSx = %d\r\n", volatile_packet.imu_dps.dps_x);
      //Serial.printf("\r\nDPSy = %d\r\n", volatile_packet.imu_dps.dps_y);
      //Serial.printf("\r\nDPS  = %d\r\n", volatile_packet.imu_dps.dps_z);
    }

    if(messageId == ANGLE_ID)
    {
      memcpy(&volatile_packet.Angle, (Angle_t *)messageData, len);
      //Serial.printf("\r\nAngle Roll = %d\r\n", volatile_packet.Angle.Roll);
      //Serial.printf("\r\nAngle Pitch = %d\r\n", volatile_packet.Angle.Pitch);
    }

    /* GPS/TELEMETRY DATA */
    if(messageId == LAT_ID)
    {
      memcpy(&volatile_packet.latitude, (double *)messageData, len);
      //Serial.println(volatile_packet.latitude);
    }

    if(messageId == LNG_ID)
    {
      memcpy(&volatile_packet.longitude, (double *)messageData, len);
      //Serial.println(volatile_packet.longitude);
    }

    if(messageId == SOT_ID)
    {
      memcpy(&volatile_packet.SOT, (uint8_t *)messageData, len);
      //Serial.println(volatile_packet.SOT);
    }
  }
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
  if(strlen(simPIN) && modem.getSimStatus() != 3)
  {
    modem.simUnlock(simPIN);
  }

  Serial.print("Waiting for network...");
  if(!modem.waitForNetwork(240000L))
  {
    Serial.println("fail");
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
    if(!mqttClient.connected())
    {
      volatile_packet.SOT &= ~(0x01); // disable online flag (00000000)
      gsmReconnect();
    }

    publishPacket();

    /*for (unsigned long start = millis(); millis() - start < timeoutTime;)
    {
      while (neogps.available())
      {
        if (gps.encode(neogps.read()))
        {
          volatile_packet.latitude = gps.location.lat();
          volatile_packet.longitude = gps.location.lng();
        }
      }
    }*/

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
      volatile_packet.SOT |= 0x01; // enable online flag (00000001)

      /* Subscribe to topics */
      mqttClient.subscribe("/esp-test");
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      Serial.print("Failed with state");
      Serial.println(mqttClient.state());
      volatile_packet.SOT &= ~(0x01); // disable online flag (00000000)
      delay(2000); 
    }
  }
}

void publishPacket()
{
  StaticJsonDocument<300> doc;

  //doc["accx"] = (volatile_packet.imu_acc.acc_x*0.061)/1000;
  doc["accx"] = Debug_accx;
  doc["accy"] = (volatile_packet.imu_acc.acc_y*0.061)/1000; 
  doc["accz"] = (volatile_packet.imu_acc.acc_z*0.061)/1000; 
  doc["dpsx"] = volatile_packet.imu_dps.dps_x;
  doc["dpsy"] = volatile_packet.imu_dps.dps_y;
  doc["dpsz"] = volatile_packet.imu_dps.dps_z;
  //doc["roll"] = volatile_packet.Angle.Roll;
  //doc["pitch"] = volatile_packet.Angle.Pitch;
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
  doc["fuel_level"] = volatile_packet.fuel;
  //doc["timestamp"] = millis();
  //doc["fuel_level"] = volatile_packet.fuel;
  //doc["timestamp"] = volatile_packet.timestamp;

  memset(msg, 0, sizeof(msg));
  serializeJson(doc, msg);
  mqttClient.publish("/logging", msg);
}

/* Interrupts routine */
void IRAM_ATTR can_ISR()
{
  detachInterrupt(digitalPinToInterrupt(CAN_INTERRUPT));
  l_state = CAN_STATE;
}

void ticker1HzISR()
{
  saveFlag = true;
  state_buffer.push(SOT_ST);
  //state_buffer.push(GPS_ST);
  //state_buffer.push(DEBUG_ST);
}