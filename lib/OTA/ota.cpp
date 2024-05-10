#include "ota.h"

boolean device = false;
String esp32_local_ip = "";
WebServer server(80);
WiFiManager wifiManager; // Objeto de manipulação do wi-fi

void setup_wifi_callback_OTA(String __IP)
{
  esp32_local_ip = __IP;
  // callback para quando entra em modo de configuração AP
  wifiManager.setAPCallback(configModeCallback);
  
  // callback para quando se conecta em uma rede, ou seja, quando passa a trabalhar em modo estação
  // wifiManager.setSaveConfigCallback(saveConfigCallback);

  start_server();
}

void configModeCallback(WiFiManager *ConectedDevice)
{
  device = true;
  Serial.println("Entrou no modo de configuração");
  Serial.println(esp32_local_ip);
  Serial.println(ConectedDevice->getConfigPortalSSID()); // imprime o SSID criado da rede
}

// Callback que indica que salvamos uma nova rede para se conectar (modo estação)
/*void saveConfigCallback()
{
  Serial.println("Configuração salva");
}*/

/* Códigos da página que será aberta no browser
   (quando comunicar via browser com o ESP32)
   Esta página exigirá um login e senha, de modo que somente
   quem tenha estas informações consiga atualizar o firmware
   do ESP32 de forma OTA */
const char *loginIndex =
    "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
    "<tr>"
    "<td colspan=2>"
    "<center><font size=4><b>ESP32 - identifique-se</b></font></center>"
    "<br>"
    "</td>"
    "<br>"
    "<br>"
    "</tr>"
    "<td>Login:</td>"
    "<td><input type='text' size=25 name='userid'><br></td>"
    "</tr>"
    "<br>"
    "<br>"
    "<tr>"
    "<td>Senha:</td>"
    "<td><input type='Password' size=25 name='pwd'><br></td>"
    "<br>"
    "<br>"
    "</tr>"
    "<tr>"
    "<td><input type='submit' onclick='check(this.form)' value='Identificar'></td>"
    "</tr>"
    "</table>"
    "</form>"
    "<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Login ou senha inválidos')"
    "}"
    "}"
    "</script>";

const char *serverIndex =
    "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
    "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
    "<input type='file' name='update'>"
    "<input type='submit' value='Update'>"
    "</form>"
    "<div id='prg'>Progresso: 0%</div>"
    "<script>"
    "$('form').submit(function(e){"
    "e.preventDefault();"
    "var form = $('#upload_form')[0];"
    "var data = new FormData(form);"
    " $.ajax({"
    "url: '/update',"
    "type: 'POST',"
    "data: data,"
    "contentType: false,"
    "processData:false,"
    "xhr: function() {"
    "var xhr = new window.XMLHttpRequest();"
    "xhr.upload.addEventListener('progress', function(evt) {"
    "if (evt.lengthComputable) {"
    "var per = evt.loaded / evt.total;"
    "$('#prg').html('Progresso: ' + Math.round(per*100) + '%');"
    "}"
    "}, false);"
    "return xhr;"
    "},"
    "success:function(d, s) {"
    "console.log('Sucesso!')"
    "},"
    "error: function (a, b, c) {"
    "}"
    "});"
    "});"
    "</script>";

void start_server()
{
  /* Configfura as páginas de login e upload de firmware OTA */
  server.on("/", HTTP_GET, []()
  {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });

  server.on("/serverIndex", HTTP_GET, []()
  {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });

  /* Define tratamentos do update de firmware OTA */
  server.on("/update", HTTP_POST, []()
  {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();

    if(upload.status==UPLOAD_FILE_START)
    {
      /* Inicio do upload de firmware OTA */
      //wifiManager.resetSettings();
      Serial.printf("Update: %s\r\n", upload.filename.c_str());
      if(!Update.begin(UPDATE_SIZE_UNKNOWN))
        Update.printError(Serial);
    }

    else if(upload.status==UPLOAD_FILE_WRITE)
    {
      /* Escrevendo firmware enviado na flash do ESP32 */
      if(Update.write(upload.buf, upload.currentSize) != upload.currentSize)
        Update.printError(Serial);
    }

    else if(upload.status==UPLOAD_FILE_END)
    {
      /* Final de upload */
      if(Update.end(true))
        Serial.printf("Sucesso no update de firmware: %u\r\nReiniciando ESP32...\r\n", upload.totalSize);
      else
        Update.printError(Serial);
    }
  });

  server.begin();
}

void HandleClient()
{
  Serial.printf("Device is conected? Sim[1] Nao[0]: %i\r\n", device_conected_in_esp32());
  if(device_conected_in_esp32())
    server.handleClient();
  vTaskDelay(1);
}

bool device_conected_in_esp32()
{
  device = WiFi.status()==WL_CONNECTED;

  return device && WiFi.status()==WL_CONNECTED;
}
