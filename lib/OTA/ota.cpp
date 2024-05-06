#include "ota.h"

String esp32_local_ip = "";
WiFiServer server(80);
WiFiManager wifiManager;//Objeto de manipulação do wi-fi

void setup_wifi_callback_OTA(String __IP)
{
  esp32_local_ip = __IP;
  //callback para quando entra em modo de configuração AP
  wifiManager.setAPCallback(configModeCallback);
  //callback para quando se conecta em uma rede, ou seja, quando passa a trabalhar em modo estação
  //wifiManager.setSaveConfigCallback(saveConfigCallback);

  server.begin();
}

void configModeCallback(WiFiManager* ConectedDevice)
{
  Serial.println("Entrou no modo de configuração");
  Serial.println(esp32_local_ip);
  Serial.println(ConectedDevice->getConfigPortalSSID()); //imprime o SSID criado da rede
}

//Callback que indica que salvamos uma nova rede para se conectar (modo estação)
/*void saveConfigCallback() 
{
  Serial.println("Configuração salva");
}*/