#include "SD_state_machine.h"

// Define timeout time in milliseconds,0 (example: 2000ms = 2s)
const long timeoutTime = 3000;
bool mounted = false;
char file_name[20];
File root; 
File dataFile;
/* Debug Variables */
mqtt_packet_t storage_data;
boolean savingBlink = false;
boolean saveFlag = false;

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

mqtt_packet_t setupVolatilePacket()
{ 
  mqtt_packet_t t;
  memset(&t, 0x00, sizeof(mqtt_packet_t));
  return t;
}

bool start_SD_device()
{
  do { Serial.println("Mount SD..."); } while(!sdConfig() && millis() < timeoutTime);

  if(!mounted) Serial.println("SD mounted error!!"); 
  else sdSave(true);

  return mounted;
}

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
    if(!entry)
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
    dataFile.println(set);
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
    }
    
    else
    {
      // imu
      dataString += String((storage_data.imu_acc.acc_x*0.061)/1000);
      dataString += ",";
      dataString += String((storage_data.imu_acc.acc_y*0.061)/1000);
      dataString += ",";
      dataString += String((storage_data.imu_acc.acc_z*0.061)/1000);
      dataString += ",";
      dataString += String(storage_data.imu_dps.dps_x);
      dataString += ",";
      dataString += String(storage_data.imu_dps.dps_y);
      dataString += ",";
      dataString += String(storage_data.imu_dps.dps_z);
      dataString += ",";
      dataString += String(storage_data.Angle.Roll);
      dataString += ",";
      dataString += String(storage_data.Angle.Pitch);

      dataString += ",";
      dataString += String(storage_data.rpm);
      dataString += ",";
      dataString += String(storage_data.speed);
      dataString += ",";
      dataString += String(storage_data.temperature);
      dataString += ",";
      dataString += String(storage_data.SOC);
      dataString += ",";
      dataString += String(storage_data.cvt);
      dataString += ",";
      //dataString += String(storage_data.fuel);
      //dataString += ",";
      dataString += String(storage_data.volt);
      dataString += ",";
      dataString += String(storage_data.current);
      dataString += ",";
      dataString += String(storage_data.flags);
      dataString += ",";
      dataString += String(storage_data.latitude);
      dataString += ",";
      dataString += String(storage_data.longitude);
      dataString += ",";
      dataString += String(storage_data.timestamp);
    }

  return dataString;
}

void Check_SD_for_storage()
{
  storage_data = update_packet();
  if(saveFlag)
  {
    sdSave(false);
    saveFlag = false;    
  }
}

/* Ticker routine */
Ticker ticker40Hz;

void setup_SD_ticker()
{ 
  ticker40Hz.attach(0.025, ticker40HzISR);
}

void ticker40HzISR()
{
  saveFlag = true; // set to true when the ECU store the CAN datas
}