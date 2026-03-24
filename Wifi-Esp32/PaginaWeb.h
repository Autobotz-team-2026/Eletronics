#ifndef PAGINA_WEB_H
#define PAGINA_WEB_H

#include <Arduino.h>
#include <WiFiManager.h> 

extern WiFiManager wm;
extern unsigned long interval;

void handleRoute() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{text-align:center;font-family:sans-serif;} .btn{display:block;width:80%;padding:20px;margin:15px auto;font-size:1.5rem;color:white;text-decoration:none;border-radius:10px;}";
  html += ".up{background:#4CAF50;} .down{background:#f44336;}</style></head><body>";
  html += "<h2>Controle do LED</h2>";
  html += "<p>Intervalo Atual: <b>" + String(interval) + " ms</b></p>";
  html += "<a class='btn up' href='/mais'>Aumentar Tempo</a>";
  html += "<a class='btn down' href='/menos'>Diminuir Tempo</a>";
  html += "<br><a href='/'>Voltar ao Menu</a></body></html>";
  
  wm.server->send(200, "text/html", html);
}

void handleMais() {
  interval += 200;
  handleRoute(); // Atualiza a página com o novo valor
}

void handleMenos() {
  if (interval > 200) interval -= 200;
  handleRoute();
}

void setupCustomRoutes() {
  wm.server->on("/controle", handleRoute);
  wm.server->on("/mais", handleMais);
  wm.server->on("/menos", handleMenos);
}

#endif