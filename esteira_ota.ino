#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h> // Biblioteca do Markus Sattler
#include <ArduinoOTA.h> 

// ==========================================
// CONFIGURAÇÕES GERAIS E PINOS
// ==========================================
#define LED_PIN 97

// Pinos Motor 1 (Esquerda) e Motor 2 (Direita)
#define INA_M1 16
#define INB_M1 15
#define EN_M1 7
#define INA_M2 5
#define INB_M2 6
#define EN_M2 4

// Parâmetros Físicos e PWM
const float WHEEL_BASE = 0.2125; //metros
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

// ==========================================
// VARIÁVEIS MICRO-ROS
// ==========================================
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Serial0.println("[ERRO ROS] Falha critica no micro-ROS.");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

int calculate_pwm_with_deadband(float target_velocity) {
  float abs_vel = abs(target_velocity);
  if (abs_vel < 0.01) return 0;
  if (abs_vel > MAX_SPEED) abs_vel = MAX_SPEED;
  return (int)(MIN_PWM + (abs_vel / MAX_SPEED) * (MAX_PWM - MIN_PWM));
}

// ==========================================
// CALLBACK DO ROS (Roda no Core 0)
// ==========================================
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * m = (const geometry_msgs__msg__Twist *)msgin;
  if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(5)) == pdTRUE) {
    sharedCmd.v = m->linear.x;
    sharedCmd.w = m->angular.z;
    sharedCmd.last_update = millis();
    xSemaphoreGive(xMutexData);
  }
}

// ==========================================
// TAREFA 1: Controle de Hardware (Core 1)
// ==========================================
void Task_Control(void *pvParameters) {
  Serial0.begin(115200);
  Serial0.println(">> [CORE 1] Hardware Iniciado");

  pinMode(LED_PIN, OUTPUT);
  pinMode(INA_M1, OUTPUT); pinMode(INB_M1, OUTPUT);
  pinMode(INA_M2, OUTPUT); pinMode(INB_M2, OUTPUT);
  ledcAttach(EN_M1, PWM_FREQ, PWM_RES);
  ledcAttach(EN_M2, PWM_FREQ, PWM_RES);

  digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, LOW); ledcWrite(EN_M1, 0);
  digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, LOW); ledcWrite(EN_M2, 0);

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
      ledcWrite(EN_M1, 0); ledcWrite(EN_M2, 0);
      digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, LOW);
      digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, LOW);
      digitalWrite(LED_PIN, LOW);
      //Serial0.println("[WATCHDOG] Sem sinal do ROS ou Wi-Fi! Motores desligados.");
    }
    else {
      digitalWrite(LED_PIN, (v_local == 0 && w_local == 0) ? LOW : HIGH);
      float v_left_raw = v_local - (w_local * WHEEL_BASE / 2.0);
      float v_right_raw = v_local + (w_local * WHEEL_BASE / 2.0);

      float v_left = (v_left_raw * CALIB_00) + (v_right_raw * CALIB_01);
      float v_right = (v_left_raw * CALIB_10) + (v_right_raw * CALIB_11);

      int pwm_left = calculate_pwm_with_deadband(v_left);
      int pwm_right = calculate_pwm_with_deadband(v_right);

      if (v_left > 0) { digitalWrite(INA_M1, HIGH); digitalWrite(INB_M1, LOW); }
      else if (v_left < 0) { digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, HIGH); }
      else { digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, LOW); }
      ledcWrite(EN_M1, pwm_left);

      if (v_right > 0) { digitalWrite(INA_M2, HIGH); digitalWrite(INB_M2, LOW); }
      else if (v_right < 0) { digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, HIGH); }
      else { digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, LOW); }
      ledcWrite(EN_M2, pwm_right);
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
    // NOTA: A porta padrão do servidor Markus Sattler é a 81
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
      Serial0.printf(">> [WS] Desconectado! ID: %u\n", num);
      if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(5)) == pdTRUE) {
        sharedCmd.v = 0; sharedCmd.w = 0;
        xSemaphoreGive(xMutexData);
      }
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial0.printf(">> [WS] Conectado! ID: %u IP: %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
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
        if (dir == "F") target_v = speed;
        else if (dir == "T") target_v = -speed;
        else if (dir == "E") target_w = speed;
        else if (dir == "D") target_w = -speed;

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
  WiFi.mode(WIFI_STA);
  WiFi.begin("ConexaoFacil", "12345678");
  WiFi.setSleep(false);

  Serial0.print(">> [WEB] Conectando ao Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial0.print(".");
  }
  Serial0.println();
  Serial0.print(">> [WEB] IP: http://"); Serial0.println(WiFi.localIP());

  // Inicia o Servidor HTTP
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", html_page);
  });
  server.begin();

  // Inicia o Servidor WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // ==========================================
  // CONFIGURAÇÃO DO OTA
  // ==========================================
  ArduinoOTA.setHostname("autobotz"); 
  ArduinoOTA.setPassword("12345678");

  ArduinoOTA.onStart([]() {
    Serial0.println("\n>> [OTA] Iniciando Atualizacao via Rede...");
    
    // SEGURANÇA: Zera a velocidade na memória compartilhada
    if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(10)) == pdTRUE) {
        sharedCmd.v = 0; sharedCmd.w = 0;
        xSemaphoreGive(xMutexData);
    }
    // SEGURANÇA: Corta os motores fisicamente na marra
    ledcWrite(EN_M1, 0); ledcWrite(EN_M2, 0);
    digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, LOW);
    digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, LOW);
  });

  ArduinoOTA.onEnd([]() {
    Serial0.println("\n>> [OTA] Atualizacao Concluida! Reiniciando...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial0.printf(">> [OTA] Progresso: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.begin();
  // ==========================================

  while (1) {
    server.handleClient();
    webSocket.loop();
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ==========================================
// TAREFA 3: Comunicação micro-ROS (Core 0)
// ==========================================
void Task_MicroROS(void *pvParameters) {
  set_microros_transports();
  vTaskDelay(pdMS_TO_TICKS(2000));

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==========================================
// SETUP INICIAL
// ==========================================
void setup() {
  xMutexData = xSemaphoreCreateMutex();

  if (xMutexData != NULL) {
    xTaskCreatePinnedToCore(Task_Control, "Controle_HW", 4096, NULL, 2, NULL, 1);
    // xTaskCreatePinnedToCore(Task_MicroROS, "Micro_ROS", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(Task_WebServer, "Web_Server", 4096, NULL, 1, NULL, 0);
  }
}

void loop() {
  vTaskDelete(NULL);
}