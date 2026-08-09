#include <WiFi.h>
#include <WiFiManager.h>
#include "OTA.h"
#include "PaginaWeb.h"  

#define LED_PIN 2
unsigned long previousMillis = 0;
unsigned long interval = 200; 
bool ledState = LOW;


void setup() {
    
    setupOTA(); 

    Serial.println("\n\n========================================");
    Serial.println("      🚀 SISTEMA AUTOBOTZ INICIADO      ");
    Serial.println("========================================");
    
    // 1. Força o WiFiManager a iniciar o servidor web internamente
    wm.startWebPortal(); 

    // 2. Agora que o servidor existe, configuramos as rotas customizadas
    setupCustomRoutes(); 

    wm.setSaveConfigCallback([](){
        setupCustomRoutes();
        Serial.println("Rotas reativadas após salvar configuração!");
    });

    // 3. Configura o menu
    std::vector<const char *> menu = {"wifi", "info", "custom", "sep", "restart"};
    wm.setMenu(menu);
    wm.setCustomMenuHTML("<a href='/controle'><button>CONTROLE LED</button></a><br/>");
    
    pinMode(LED_PIN, OUTPUT);

    if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[ STATUS DE CONEXÃO ]");
    Serial.print("  🌐 Rede:      "); Serial.println(WiFi.SSID());
    Serial.print("  📍 IP Local:  "); Serial.println(WiFi.localIP());
    Serial.print("  📡 Hostname:  "); Serial.println(ArduinoOTA.getHostname());
    Serial.println("----------------------------------------");
    Serial.println("✅ Servidor Web e OTA prontos!");
    Serial.println("========================================\n");
    }
}

void loop() {

   // wm.resetSettings();
    ArduinoOTA.handle();
    wm.process();

    if (WiFi.status() != WL_CONNECTED) {
         wm.startConfigPortal("RECONECTAR_AUTOBOTZ", "12345678");
    }

    if (WiFi.status() == WL_CONNECTED && !wm.getWebPortalActive()) {
        Serial.println("SITE REABERTO");
        wm.startWebPortal();
        setupCustomRoutes();
    }

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }
}