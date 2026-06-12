#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoOTA.h>

// ==========================================
// CONFIGURAÇÕES GERAIS E PINOS
// ==========================================
#define LED_PIN 97

// --- Pinos Motor 1 (Esquerda) - L298N ---
#define IN1_M1 5
#define IN2_M1 4
#define EN_M1 6

// --- Pinos Motor 2 (Direita) - BTS7960 ---
#define RPWM_M2 17
#define LPWM_M2 19
#define R_EN_M2 18
#define L_EN_M2 20

// Parâmetros Físicos e PWM
const float WHEEL_BASE = 0.2125; // metros
const float MAX_SPEED = 0.3;
const uint32_t PWM_FREQ = 2000;
const uint8_t PWM_RES = 11;
const float MAX_PWM = 1605.0;
const float MIN_PWM = 480.0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;

// Matriz de Calibração dos Motores
const float CALIB_00 = 1.00;
const float CALIB_01 = 0.00;
const float CALIB_10 = 0.00;
const float CALIB_11 = 0.95;

// ==========================================
// MEMÓRIA COMPARTILHADA (MUTEX)
// ==========================================
struct RobotCommand {
    float v;
    float w;
    unsigned long last_update;
};

RobotCommand sharedCmd = {0.0, 0.0, 0};
SemaphoreHandle_t xMutexData = NULL;

int calculate_pwm_with_deadband(float target_velocity) {
    float abs_vel = abs(target_velocity);
    if (abs_vel < 0.01) return 0;
    if (abs_vel > MAX_SPEED) abs_vel = MAX_SPEED;
    return (int)(MIN_PWM + (abs_vel / MAX_SPEED) * (MAX_PWM - MIN_PWM));
}

// ==========================================
// TAREFA 1: Controle de Hardware (Core 1)
// ==========================================
void Task_Control(void *pvParameters) {
    Serial.begin(115200);
    Serial.println(">> [CORE 1] Hardware Iniciado");

    pinMode(LED_PIN, OUTPUT);

    // Setup Motor 1 (L298N)
    pinMode(IN1_M1, OUTPUT);
    pinMode(IN2_M1, OUTPUT);

    // Define canais PWM
    const int PWM_CHAN_LEFT = 0;
    const int PWM_CHAN_RPWM = 1;
    const int PWM_CHAN_LPWM = 2;

    // Configura PWM para Motor 1
    ledcSetup(PWM_CHAN_LEFT, PWM_FREQ, PWM_RES);
    ledcAttachPin(EN_M1, PWM_CHAN_LEFT);

    digitalWrite(IN1_M1, LOW);
    digitalWrite(IN2_M1, LOW);
    ledcWrite(PWM_CHAN_LEFT, 0);

    // Setup Motor 2 (BTS7960)
    pinMode(R_EN_M2, OUTPUT);
    pinMode(L_EN_M2, OUTPUT);
    digitalWrite(R_EN_M2, HIGH);
    digitalWrite(L_EN_M2, HIGH);

    // Configura PWM para Motor 2
    ledcSetup(PWM_CHAN_RPWM, PWM_FREQ, PWM_RES);
    ledcAttachPin(RPWM_M2, PWM_CHAN_RPWM);

    ledcSetup(PWM_CHAN_LPWM, PWM_FREQ, PWM_RES);
    ledcAttachPin(LPWM_M2, PWM_CHAN_LPWM);

    ledcWrite(PWM_CHAN_RPWM, 0);
    ledcWrite(PWM_CHAN_LPWM, 0);

    float v_local = 0.0, w_local = 0.0;
    unsigned long time_local = 0;

    while (1) {
        if (xSemaphoreTake(xMutexData, portMAX_DELAY) == pdTRUE) {
            v_local = sharedCmd.v;
            w_local = sharedCmd.w;
            time_local = sharedCmd.last_update;
            xSemaphoreGive(xMutexData);
        }

        if ((millis() - time_local) > WATCHDOG_TIMEOUT_MS) {
            ledcWrite(PWM_CHAN_LEFT, 0);
            ledcWrite(PWM_CHAN_RPWM, 0);
            ledcWrite(PWM_CHAN_LPWM, 0);
            digitalWrite(IN1_M1, LOW);
            digitalWrite(IN2_M1, LOW);
            digitalWrite(LED_PIN, LOW);
        }
        else {
            digitalWrite(LED_PIN, (v_local == 0 && w_local == 0) ? LOW : HIGH);

            float v_left_raw = v_local - (w_local * WHEEL_BASE / 2.0);
            float v_right_raw = v_local + (w_local * WHEEL_BASE / 2.0);

            float v_left = (v_left_raw * CALIB_00) + (v_right_raw * CALIB_01);
            float v_right = (v_left_raw * CALIB_10) + (v_right_raw * CALIB_11);

            int pwm_left = calculate_pwm_with_deadband(v_left);
            int pwm_right = calculate_pwm_with_deadband(v_right);

            // Motor Esquerdo (L298N)
            if (v_left > 0) {
                digitalWrite(IN1_M1, HIGH);
                digitalWrite(IN2_M1, LOW);
            }
            else if (v_left < 0) {
                digitalWrite(IN1_M1, LOW);
                digitalWrite(IN2_M1, HIGH);
            }
            else {
                digitalWrite(IN1_M1, LOW);
                digitalWrite(IN2_M1, LOW);
            }
            ledcWrite(PWM_CHAN_LEFT, pwm_left);

            // Motor Direito (BTS7960)
            if (v_right > 0) {
                ledcWrite(PWM_CHAN_LPWM, 0);
                ledcWrite(PWM_CHAN_RPWM, pwm_right);
            }
            else if (v_right < 0) {
                ledcWrite(PWM_CHAN_RPWM, 0);
                ledcWrite(PWM_CHAN_LPWM, pwm_right);
            }
            else {
                ledcWrite(PWM_CHAN_RPWM, 0);
                ledcWrite(PWM_CHAN_LPWM, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================
// PÁGINA HTML (COM WEBSOCKETS NO JAVASCRIPT)
// ==========================================
const char* html_page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>Controle do Robô</title>
  <style>
    body {
      display: flex;
      flex-direction: column;
      align-items: center;
      background-color: #222;
      color: white;
      font-family: sans-serif;
      user-select: none;
      margin: 0;
      padding-top: 20px;
    }
    .status { font-weight: bold; margin-bottom: 20px; font-size: 18px; }
    .status.on { color: #0f0; }
    .status.off { color: #f00; }

    .dpad {
      display: grid;
      grid-template-columns: 110px 110px 110px;
      grid-template-rows: 110px 110px 110px;
      gap: 15px;
      margin-top: 10px;
    }
    .btn {
        background-color: #444;
        border: 3px solid #888;
        display: flex;
        justify-content: center;
        align-items: center;
        font-size: 50px;
        font-weight: bold;
        cursor: pointer;
        transition: background-color 0.1s;
    }
    .btn:active { background-color: #00ff00; color: #000; }

    .up { grid-column: 2; grid-row: 1; border-radius: 20px 20px 8px 8px; }
    .down { grid-column: 2; grid-row: 3; border-radius: 8px 8px 20px 20px; }
    .left { grid-column: 1; grid-row: 2; border-radius: 20px 8px 8px 20px; }
    .right { grid-column: 3; grid-row: 2; border-radius: 8px 20px 20px 8px; }
    .center { grid-column: 2; grid-row: 2; border-radius: 50%; background-color: #cc0000; font-size: 22px; border-color: #ff4444; }
    .center:active { background-color: #ff0000; color: white; }

    .slider-container { margin-top: 50px; width: 90%; max-width: 350px; text-align: center; font-size: 20px; }
    input[type=range] { width: 100%; height: 40px; }
    </style>
    </head>
    <body>
    <h2>Robô Esteira</h2>
    <div id="connStatus" class="status off">Conectando...</div>

    <div class="dpad">
    <div class="btn up" onmousedown="cmd('F')" onmouseup="cmd('P')" onmouseleave="cmd('P')" ontouchstart="cmd('F')" ontouchend="cmd('P')">⬆</div>
    <div class="btn left" onmousedown="cmd('E')" onmouseup="cmd('P')" onmouseleave="cmd('P')" ontouchstart="cmd('E')" ontouchend="cmd('P')">⬅</div>
    <div class="btn center" onclick="cmd('P')">STOP</div>
    <div class="btn right" onmousedown="cmd('D')" onmouseup="cmd('P')" onmouseleave="cmd('P')" ontouchstart="cmd('D')" ontouchend="cmd('P')">➡</div>
    <div class="btn down" onmousedown="cmd('T')" onmouseup="cmd('P')" onmouseleave="cmd('P')" ontouchstart="cmd('T')" ontouchend="cmd('P')">⬇</div>
    </div>

    <div class="slider-container">
    <label>Velocidade Máxima: <span id="spd_val">50</span>%</label>
    <br><br>
    <input type="range" id="speed" min="10" max="100" value="50" oninput="document.getElementById('spd_val').innerText = this.value">
    </div>

    <script>
    var gateway = `ws://${window.location.hostname}:81/`;
    var websocket;
    var timer;
    var currentDir = 'P';

    window.addEventListener('load', onLoad);
    function onLoad(event) { initWebSocket(); }

    function initWebSocket() {
        websocket = new WebSocket(gateway);
        websocket.onopen    = onOpen;
        websocket.onclose   = onClose;
    }

    function onOpen(event) {
        let statusTxt = document.getElementById('connStatus');
        statusTxt.innerText = "ONLINE";
        statusTxt.className = "status on";
    }

    function onClose(event) {
        let statusTxt = document.getElementById('connStatus');
        statusTxt.innerText = "OFFLINE - Reconectando...";
        statusTxt.className = "status off";
        setTimeout(initWebSocket, 2000);
    }

    function cmd(dir) {
        clearInterval(timer);
        currentDir = dir;

        if (dir === 'P') {
            enviar('P');
        } else {
            enviar(dir);
            timer = setInterval(function() { enviar(currentDir); }, 100);
        }
    }

    function enviar(dir) {
        if (websocket && websocket.readyState == 1) {
            var s = document.getElementById('speed').value / 100.0;
            websocket.send(dir + "," + s);
        }
    }
    </script>
    </body>
    </html>
    )rawliteral";

    // ==========================================
    // SERVIDOR WEB (Porta 80) e WEBSOCKET (Porta 81)
    // ==========================================
    WebServer server(80);
    WebSocketsServer webSocket = WebSocketsServer(81);

    void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
        switch(type) {
            case WStype_DISCONNECTED:
                Serial.printf(">> [WS] Desconectado! ID: %u\n", num);
                if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(5)) == pdTRUE) {
                    sharedCmd.v = 0; sharedCmd.w = 0;
                    xSemaphoreGive(xMutexData);
                }
                break;

            case WStype_CONNECTED: {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf(">> [WS] Conectado! ID: %u IP: %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
                break;
            }

            case WStype_TEXT: {
                String msg((char*)payload, length);
                int commaIndex = msg.indexOf(',');

                if (msg == "P") {
                    if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(5)) == pdTRUE) {
                        sharedCmd.v = 0; sharedCmd.w = 0;
                        sharedCmd.last_update = millis();
                        xSemaphoreGive(xMutexData);
                    }
                }
                else if (commaIndex > 0) {
                    String dir = msg.substring(0, commaIndex);
                    float speed = msg.substring(commaIndex + 1).toFloat();

                    float target_v = 0.0, target_w = 0.0;

                    // Fator de conversão para garantir torque suficiente na rotação
                    float angular_multiplier = 2.0 / WHEEL_BASE;

                    if (dir == "F") target_v = speed;
                    else if (dir == "T") target_v = -speed;
                    else if (dir == "E") target_w = speed * angular_multiplier;
                    else if (dir == "D") target_w = -speed * angular_multiplier;

                    if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(5)) == pdTRUE) {
                        sharedCmd.v = target_v;
                        sharedCmd.w = target_w;
                        sharedCmd.last_update = millis();
                        xSemaphoreGive(xMutexData);
                    }
                }
                break;
            }
        }
    }

    void Task_WebServer(void *pvParameters) {
        // Configura o ESP32 para criar a própria rede (Access Point)
        WiFi.mode(WIFI_AP);
        WiFi.softAP("Robo_Autobotz", "12345678"); // Você pode mudar o nome da rede e a senha aqui
        WiFi.setSleep(false);

        Serial.println();
        Serial.println(">> [WEB] Ponto de acesso Wi-Fi criado!");
        Serial.print(">> [WEB] IP para acessar no navegador: http://");
        Serial.println(WiFi.softAPIP());

        server.on("/", HTTP_GET, []() {
            server.send(200, "text/html", html_page);
        });
        server.begin();

        webSocket.begin();
        webSocket.onEvent(webSocketEvent);

        // ==========================================
        // CONFIGURAÇÃO DO OTA
        // ==========================================
        ArduinoOTA.setHostname("autobotz");
        ArduinoOTA.setPassword("12345678");

        ArduinoOTA.onStart([]() {
            Serial.println("\n>> [OTA] Iniciando Atualizacao via Rede...");

            if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(10)) == pdTRUE) {
                sharedCmd.v = 0; sharedCmd.w = 0;
                xSemaphoreGive(xMutexData);
            }

            // SEGURANÇA: Corta os motores físicos no L298N e no BTS7960
            ledcWrite(EN_M1, 0);
            ledcWrite(RPWM_M2, 0);
            ledcWrite(LPWM_M2, 0);
            digitalWrite(IN1_M1, LOW);
            digitalWrite(IN2_M1, LOW);
        });

        ArduinoOTA.onEnd([]() {
            Serial.println("\n>> [OTA] Atualizacao Concluida! Reiniciando...");
        });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf(">> [OTA] Progresso: %u%%\r", (progress / (total / 100)));
        });

        ArduinoOTA.begin();

        while (1) {
            server.handleClient();
            webSocket.loop();
            ArduinoOTA.handle();
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    // ==========================================
    // SETUP INICIAL
    // ==========================================
    void setup() {
        xMutexData = xSemaphoreCreateMutex();

        if (xMutexData != NULL) {
            xTaskCreatePinnedToCore(Task_Control, "Controle_HW", 4096, NULL, 2, NULL, 1);
            xTaskCreatePinnedToCore(Task_WebServer, "Web_Server", 4096, NULL, 1, NULL, 0);
        }
    }

    void loop() {
        vTaskDelete(NULL);
    }