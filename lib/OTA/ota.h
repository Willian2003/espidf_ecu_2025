#ifndef OTA_H
#define OTA_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <DNSServer.h> 
#include <WebServer.h>
#include <Update.h>

void setup_wifi_callback_OTA(String __IP);

/* Wifi callbacks */
void configModeCallback(WiFiManager* ConectedDevice);
//void saveConfigCallback(void);

#endif