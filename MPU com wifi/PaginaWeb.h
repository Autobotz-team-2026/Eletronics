#ifndef PAGINA_WEB_H
#define PAGINA_WEB_H

#include <Arduino.h>
#include <WiFiManager.h> 
#include <WebServer.h>        
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Kalman.h>

struct MPU_Data {
  Adafruit_MPU6050 mpu;
  Kalman kalman;
  float anguloFiltrado = 0.0;
  double timer = 0;
};

// Avisando o compilador que estas variáveis foram criadas em outro lugar
extern WiFiManager wm;
extern MPU_Data m1, m2, m3;
extern SemaphoreHandle_t xMutex;

// --- CRIA O SERVIDOR INDEPENDENTE ---
WebServer server(80);

// --- HTML DO DASHBOARD ---
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Dashboard MPU - Kalman</title>
    <style>
        body { font-family: sans-serif; background: #121212; color: #00ff41; text-align: center; }
        .container { margin-top: 50px; }
        .card { background: #1e1e1e; border: 1px solid #333; padding: 20px; margin: 10px; display: inline-block; width: 250px; border-radius: 10px; }
        h1 { color: #fff; }
        .valor { font-size: 2em; font-weight: bold; }
    </style>
</head>
<body>
    <h1>Monitoramento em Tempo Real</h1>
    <div class="container">
        <div class="card"><h2>MPU 1</h2><div class="valor" id="v1">0.0</div><div>GRAUS</div></div>
        <div class="card"><h2>MPU 2</h2><div class="valor" id="v2">0.0</div><div>GRAUS</div></div>
        <div class="card"><h2>MPU 3</h2><div class="valor" id="v3">0.0</div><div>GRAUS</div></div>
    </div>
    <script>
        setInterval(function() {
            fetch('/data').then(response => response.json()).then(data => {
                document.getElementById('v1').innerHTML = data.m1.toFixed(1);
                document.getElementById('v2').innerHTML = data.m2.toFixed(1);
                document.getElementById('v3').innerHTML = data.m3.toFixed(1);
            });
        }, 200); // Atualiza a cada 200ms
    </script>
</body>
</html>
)=====";

// --- FUNÇÕES DO SERVIDOR ---
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleData() {
  String json = "{";
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    json += "\"m1\":" + String(m1.anguloFiltrado) + ",";
    json += "\"m2\":" + String(m2.anguloFiltrado) + ",";
    json += "\"m3\":" + String(m3.anguloFiltrado);
    xSemaphoreGive(xMutex);
  }
  json += "}";
  server.send(200, "application/json", json);
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.begin(); 
  Serial.println("Servidor do Dashboard iniciado na porta 80.");
}

void loopWebServer() {
  server.handleClient(); 
}

void stopWebServer() {
  server.stop(); // Encerra o servidor e libera a porta 80
  Serial.println("Servidor do Dashboard pausado. Porta 80 liberada.");
}

#endif