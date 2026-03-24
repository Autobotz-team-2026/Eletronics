#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h> 
#include "PaginaWeb.h"

WiFiManager wm; 

void saveConfigCallback() {
  Serial.println("Novas configurações salvas com sucesso!");
}

void setupOTA() {


  Serial.begin(115200);
  Serial.println("\n--- Iniciando Sistema ---");


  wm.setDebugOutput(true);
  wm.setConfigPortalTimeout(180);
  wm.setConnectTimeout(30); 
  
  // Tenta conectar. Se não conseguir, abre o Portal "RECONECTAR_AUTOBOTZ"
  Serial.println("Tentando conectar ao WiFi...");
  wm.setConfigPortalBlocking(false);
  wm.setSaveConfigCallback(saveConfigCallback);

  if (!wm.autoConnect("AUTOBOTZ", "12345678")) {
    Serial.println("Falha na conexão. O portal está aberto, mas o sistema seguirá tentando...");
  } else {
    Serial.println("Conectado com sucesso!");
  }

  ArduinoOTA.setHostname("AUTOBOTZ"); 
  ArduinoOTA.setPassword("12345678");

  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("OTA: Iniciando atualização de " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA: Finalizado com sucesso");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progresso: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Erro[%u]: ", error);
  });

  ArduinoOTA.begin();
  
  Serial.print("IP Atual: ");
  Serial.println(WiFi.localIP());
}