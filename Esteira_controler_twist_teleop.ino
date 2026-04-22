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

// ==========================================
// CONFIGURAÇÕES GERAIS E PINOS
// ==========================================
#define LED_PIN 97

// Pinos Motor 1 (Esquerda) e Motor 2 (Direita)
#define INA_M1 7
#define INB_M1 15
#define EN_M1 16
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
const float CALIB_00 = 1.00; // Multiplicador direto da roda esquerda
const float CALIB_01 = 0.00; // Influência da direita na esquerda
const float CALIB_10 = 0.00; // Influência da esquerda na direita
const float CALIB_11 = 0.95; // Multiplicador direto da roda direita

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
    Serial0.println("[ERRO ROS] Falha critica no micro-ROS. Reinicie a placa.");
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
// TAREFA 1: Controle de Hardware (Core 1 / UART)
// ==========================================
void Task_Control(void *pvParameters) {
  Serial0.begin(115200);
  Serial0.println(">> [CORE 1] Sistema de Motores, Matriz e Watchdog Iniciado");

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
    // 1. TENTA LER A MEMÓRIA COM SEGURANÇA
    if (xSemaphoreTake(xMutexData, portMAX_DELAY) == pdTRUE) {
      v_local = sharedCmd.v;
      w_local = sharedCmd.w;
      time_local = sharedCmd.last_update;
      xSemaphoreGive(xMutexData);
    }

    // 2. VERIFICA WATCHDOG
    if ((millis() - time_local) > WATCHDOG_TIMEOUT_MS) {
      ledcWrite(EN_M1, 0); ledcWrite(EN_M2, 0);
      digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, LOW);
      digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, LOW);
      digitalWrite(LED_PIN, LOW);
      Serial0.println("[WATCHDOG] Sem sinal do ROS ou Wi-Fi! Motores desligados.");
    }
    else {
      // 3. MATRIZ DE CALIBRAÇÃO E PWM
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

      if(v_local != 0 || w_local != 0) {
        Serial0.printf("[MOTOR] V: %.2f | W: %.2f | PWM_E: %d | PWM_D: %d\n", v_local, w_local, pwm_left, pwm_right);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Roda a 50Hz
  }
}

// ==========================================
// TAREFA 2: Servidor Web Wi-Fi (Core 0)
// ==========================================
WebServer server(80);

const char* html_page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>Controle do Robô</title>
  <style>
    body { display: flex; flex-direction: column; align-items: center; background-color: #222; color: white; font-family: sans-serif; user-select: none; }
    .dpad { display: grid; grid-template-columns: 80px 80px 80px; grid-template-rows: 80px 80px 80px; gap: 10px; margin-top: 50px; }
    .btn { background-color: #444; border: 2px solid #888; border-radius: 10px; display: flex; justify-content: center; align-items: center; font-size: 30px; font-weight: bold; cursor: pointer; }
    .btn:active { background-color: #00ff00; }
    .up { grid-column: 2; grid-row: 1; border-radius: 10px 10px 0 0; }
    .down { grid-column: 2; grid-row: 3; border-radius: 0 0 10px 10px; }
    .left { grid-column: 1; grid-row: 2; border-radius: 10px 0 0 10px; }
    .right { grid-column: 3; grid-row: 2; border-radius: 0 10px 10px 0; }
    .center { grid-column: 2; grid-row: 2; border-radius: 50%; background-color: #f00; }
    .slider-container { margin-top: 40px; width: 80%; text-align: center; }
    input[type=range] { width: 100%; height: 30px; }
  </style>
</head>
<body>
  <h2>Robô Esteira</h2>
  <div class="dpad">
    <div class="btn up" onmousedown="cmd('F')" onmouseup="cmd('S')" ontouchstart="cmd('F')" ontouchend="cmd('S')">'▲'</div>
    <div class="btn left" onmousedown="cmd('L')" onmouseup="cmd('S')" ontouchstart="cmd('L')" ontouchend="cmd('S')">'◀'</div>
    <div class="btn center" onclick="cmd('S')">STOP</div>
    <div class="btn right" onmousedown="cmd('R')" onmouseup="cmd('S')" ontouchstart="cmd('R')" ontouchend="cmd('S')">'▶'</div>
    <div class="btn down" onmousedown="cmd('B')" onmouseup="cmd('S')" ontouchstart="cmd('B')" ontouchend="cmd('S')">'▼'</div>
    </div>
    <div class="slider-container">
    <label>Velocidade Máxima: <span id="spd_val">50</span>%</label>
    <br><br>
    <input type="range" id="speed" min="10" max="100" value="50" oninput="document.getElementById('spd_val').innerText = this.value">
    </div>
    <script>
    var timer;

    function cmd(dir) {
      // Limpa qualquer loop anterior para não acumular
      clearInterval(timer);

      if (dir === 'S') {
        // Se for STOP, envia uma vez e para o loop
        enviar(dir);
      } else {
        // Se estiver segurando um botão, envia a cada 100ms (10Hz)
        // Isso mantém o Watchdog do ESP32 sempre alimentado
        timer = setInterval(function() {
          enviar(dir);
        }, 100);
      }
    }

    function enviar(dir) {
      var s = document.getElementById('speed').value / 100.0;
      fetch('/acao?dir=' + dir + '&v=' + s);
    }

    // Segurança extra: se o mouse sair do botão ou o toque terminar, para o loop
    document.addEventListener("mouseup", () => clearInterval(timer));
    document.addEventListener("touchend", () => clearInterval(timer));
    </script>
    </body>
    </html>
    )rawliteral";

  // ==========================================
// TAREFA 2: Servidor Web Wi-Fi (Core 0)
// ==========================================
// ... (mantenha a variável 'server' e a 'html_page' exatamente como estão no seu código) ...

void Task_WebServer(void *pvParameters) {
  // 1. Configura como "Estação" (Cliente) e conecta na sua rede
  WiFi.mode(WIFI_STA);
  WiFi.begin("LucianoCL", "calculosola");
  
  Serial0.print(">> [WEB] Conectando ao Wi-Fi LucianoCL...");

  // Fica em loop esperando conectar (o vTaskDelay impede que o núcleo trave)
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial0.print(".");
  }

  Serial0.println();
  
  // Imprime o IP que o seu roteador emprestou para o robô
  IPAddress IP = WiFi.localIP();
  Serial0.print(">> [WEB] Conectado com sucesso! Acesse no celular: http://");
  Serial0.println(IP);

  // 2. Configura a página inicial (HTML)
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", html_page);
  });

  // 3. Recebe as ordens de movimento
  server.on("/acao", HTTP_GET, []() {
    String dir = server.arg("dir");
    float speed = server.arg("v").toFloat();

    float target_v = 0.0, target_w = 0.0;
    if (dir == "F") target_v = speed;
    else if (dir == "B") target_v = -speed;
    else if (dir == "L") target_w = speed;
    else if (dir == "R") target_w = -speed;

    if (xSemaphoreTake(xMutexData, pdMS_TO_TICKS(5)) == pdTRUE) {
      sharedCmd.v = target_v;
      sharedCmd.w = target_w;
      sharedCmd.last_update = millis();
      xSemaphoreGive(xMutexData);
    }
    server.send(200, "text/plain", "OK");
  });

  // 4. Inicia o servidor e entra no loop
  server.begin();
  while (1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

  // ==========================================
  // TAREFA 3: Comunicação micro-ROS (Core 0 / Nativa)
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
      // Hardware físico e Watchdog garantidos no Core 1
      xTaskCreatePinnedToCore(Task_Control, "Controle_HW", 4096, NULL, 2, NULL, 1);

      // Comunicações de Rede rodam no Core 0
      xTaskCreatePinnedToCore(Task_MicroROS, "Micro_ROS", 8192, NULL, 1, NULL, 0);
      xTaskCreatePinnedToCore(Task_WebServer, "Web_Server", 4096, NULL, 1, NULL, 0);
    }
  }

  void loop() {
    vTaskDelete(NULL);
  }
