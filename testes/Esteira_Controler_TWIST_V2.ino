#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LED_PIN 97 

// Pinos Motor 1 (Esquerda) e Motor 2 (Direita)
#define INA_M1 7
#define INB_M1 15
#define EN_M1 16
#define INA_M2 18 
#define INB_M2 19
#define EN_M2 21

// Parâmetros Físicos
const float WHEEL_BASE = 0.25; 
const float MAX_SPEED = 1.0;   
const float CALIB_LEFT = 1.0;  
const float CALIB_RIGHT = 0.95; 

// Configurações de PWM
const uint32_t PWM_FREQ = 2000; 
const uint8_t PWM_RES = 11;     
const float MAX_PWM = 2047.0;   
const float MIN_PWM = 480.0;    

// --- VARIÁVEIS GLOBAIS COMPARTILHADAS (Volatile para segurança no RTOS) ---
volatile float target_v = 0.0;
volatile float target_w = 0.0;
volatile TickType_t last_cmd_time = 0; 
const TickType_t WATCHDOG_TIMEOUT_TICKS = pdMS_TO_TICKS(500); 

// Objetos do micro-ROS
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
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

// O callback agora SÓ atualiza as variáveis. Não mexe em hardware.
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  target_v = msg->linear.x;
  target_w = msg->angular.z;
  last_cmd_time = xTaskGetTickCount(); // Atualiza o relógio
}

int calculate_pwm_with_deadband(float target_velocity) {
  float abs_vel = abs(target_velocity);
  if (abs_vel < 0.01) return 0;
  if (abs_vel > MAX_SPEED) abs_vel = MAX_SPEED;
  return (int)(MIN_PWM + (abs_vel / MAX_SPEED) * (MAX_PWM - MIN_PWM));
}

// ==========================================
// TAREFA 1: Controle de Hardware e Watchdog
// ==========================================
void Task_Control(void *pvParameters) {
  // Configuração dos pinos (feita dentro da tarefa que vai controlá-los)
  pinMode(LED_PIN, OUTPUT);
  pinMode(INA_M1, OUTPUT); pinMode(INB_M1, OUTPUT);
  pinMode(INA_M2, OUTPUT); pinMode(INB_M2, OUTPUT);
  
  ledcAttach(EN_M1, PWM_FREQ, PWM_RES);
  ledcAttach(EN_M2, PWM_FREQ, PWM_RES);
  
  digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, LOW); ledcWrite(EN_M1, 0);
  digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, LOW); ledcWrite(EN_M2, 0);

  last_cmd_time = xTaskGetTickCount();

  // Loop Infinito da Tarefa de Controle
  while (1) {
    // 1. Verifica Watchdog
    if ((xTaskGetTickCount() - last_cmd_time) > WATCHDOG_TIMEOUT_TICKS) {
      // Parada de Emergência
      ledcWrite(EN_M1, 0); ledcWrite(EN_M2, 0);
      digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, LOW);
      digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, LOW);
      digitalWrite(LED_PIN, LOW); 
    } 
    else {
      // 2. Operação Normal (Calcula e aplica PWM)
      digitalWrite(LED_PIN, (target_v == 0 && target_w == 0) ? LOW : HIGH);

      float v_left = (target_v - (target_w * WHEEL_BASE / 2.0)) * CALIB_LEFT;
      float v_right = (target_v + (target_w * WHEEL_BASE / 2.0)) * CALIB_RIGHT;

      int pwm_left = calculate_pwm_with_deadband(v_left);
      int pwm_right = calculate_pwm_with_deadband(v_right);

      // Direção Motor Esquerdo
      if (v_left > 0) { digitalWrite(INA_M1, HIGH); digitalWrite(INB_M1, LOW); } 
      else if (v_left < 0) { digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, HIGH); } 
      else { digitalWrite(INA_M1, LOW); digitalWrite(INB_M1, LOW); }

      // Direção Motor Direito
      if (v_right > 0) { digitalWrite(INA_M2, HIGH); digitalWrite(INB_M2, LOW); } 
      else if (v_right < 0) { digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, HIGH); } 
      else { digitalWrite(INA_M2, LOW); digitalWrite(INB_M2, LOW); }

      ledcWrite(EN_M1, pwm_left);
      ledcWrite(EN_M2, pwm_right);
    }
    
    // Roda o controle a 50Hz (a cada 20ms)
    vTaskDelay(pdMS_TO_TICKS(20)); 
  }
}

// ==========================================
// TAREFA 2: Comunicação micro-ROS
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

  // Loop Infinito da Tarefa ROS
  while (1) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void setup() {
  // Cria a tarefa de Controle no Núcleo 1 (Prioridade 2 - Alta)
  xTaskCreatePinnedToCore(
    Task_Control,   // Função da tarefa
    "Controle_HW",  // Nome
    2048,           // Tamanho da pilha (Stack)
    NULL,           // Parâmetros passados
    2,              // Prioridade
    NULL,           // Handle da tarefa
    1               // Executar no Core 1
  );

  // Cria a tarefa do Micro-ROS no Núcleo 0 (Prioridade 1 - Padrão)
  xTaskCreatePinnedToCore(
    Task_MicroROS,  
    "Micro_ROS",    
    4096,           // ROS precisa de mais memória de pilha
    NULL,           
    1,              // Prioridade menor que o hardware
    NULL,           
    0               // Executar no Core 0 (junto com o Wi-Fi do ESP32)
  );
}

void loop() {
  // O loop padrão do Arduino não é mais necessário, então nós o deletamos
  // para liberar a memória que ele estaria ocupando!
  vTaskDelete(NULL); 
}
