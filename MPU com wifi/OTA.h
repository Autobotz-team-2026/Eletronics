#ifndef OTA_H
#define OTA_H

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h> 

WiFiManager wm; 

void setupOTA() {
  Serial.begin(115200);
  Serial.println("\n--- Iniciando Sistema e Abrindo a configuração WIFI ---");

  wm.setDebugOutput(false);

  wm.setConnectTimeout(30);

  wm.setSaveConfigCallback([]() {
    Serial.println("Configuração salva no portal!");
    delay(1000); 
    ESP.restart();
  });
  
  if (!wm.autoConnect("AUTOBOTZ", "12345678")) {
    Serial.println("Falha ao conectar. Reinicie o ESP ou configure o Wi-Fi.");
    delay(3000);
    ESP.restart(); 
  } else {
    Serial.print("Conectado com sucesso ao Wi-Fi: "); Serial.println(WiFi.SSID());
  }

  WiFi.mode(WIFI_STA); // Força o desligamento do Ponto de Acesso (AP)
  WiFi.softAPdisconnect(true);

  // Configuração do ArduinoOTA
  ArduinoOTA.setHostname("AUTOBOTZ"); 
  ArduinoOTA.setPassword("12345678");

  ArduinoOTA.onStart([]() { Serial.println("OTA: Iniciando atualização..."); });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA: Finalizado com sucesso"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progresso: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA Erro[%u]: ", error); });

  ArduinoOTA.begin();
}

void loopOTA() {
  ArduinoOTA.handle(); 
}

#endif