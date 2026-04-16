#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LED_PIN 97 

// Pinos Motor 1 (Esquerda)
#define INA_M1 7
#define INB_M1 15
#define EN_M1 16

// Pinos Motor 2 (Direita) - Defina os pinos corretos do seu ESP32
#define INA_M2 18 
#define INB_M2 19
#define EN_M2 21

// Parâmetros Físicos do Robô (Esteiras)
const float WHEEL_BASE = 0.25; 
const float MAX_SPEED = 1.0;   

// Matriz de Calibração (Ajuste para compensar o arrasto das esteiras)
const float CALIB_LEFT = 1.0;  
const float CALIB_RIGHT = 0.95; 

// --- NOVAS CONFIGURAÇÕES DE PWM E INÉRCIA ---
const uint32_t PWM_FREQ = 2000; // 2 kHz
const uint8_t PWM_RES = 11;     // Resolução de 11 bits (0 a 2047)
const float MAX_PWM = 2047.0;   // Teto do PWM
const float MIN_PWM = 60.0;     // PWM mínimo para vencer a inércia da esteira

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Função auxiliar para mapear a velocidade aplicando a inércia
int calculate_pwm_with_deadband(float target_velocity) {
  float abs_vel = abs(target_velocity);
  
  // Se a velocidade for virtualmente zero, o PWM é zero
  if (abs_vel < 0.01) {
    return 0;
  }
  
  // Limita a velocidade ao teto máximo para evitar cálculos acima de 100%
  if (abs_vel > MAX_SPEED) {
    abs_vel = MAX_SPEED;
  }

  // Mapeia: a velocidade (0 a MAX_SPEED) passa a ser (MIN_PWM a MAX_PWM)
  float pwm_float = MIN_PWM + (abs_vel / MAX_SPEED) * (MAX_PWM - MIN_PWM);
  
  return (int)pwm_float;
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float v = msg->linear.x;
  float w = msg->angular.z;

  // 1. Cinemática Diferencial com Matriz de Calibração Integrada
  float v_left = (v - (w * WHEEL_BASE / 2.0)) * CALIB_LEFT;
  float v_right = (v + (w * WHEEL_BASE / 2.0)) * CALIB_RIGHT;

  // 2. Converter Velocidade (m/s) para PWM compensando a inércia
  int pwm_left = calculate_pwm_with_deadband(v_left);
  int pwm_right = calculate_pwm_with_deadband(v_right);

  // 3. Controle Direcional - Esteira Esquerda
  if (v_left > 0) { // Frente
    digitalWrite(INA_M1, HIGH);
    digitalWrite(INB_M1, LOW);
  } else if (v_left < 0) { // Trás
    digitalWrite(INA_M1, LOW);
    digitalWrite(INB_M1, HIGH);
  } else { // Parado
    digitalWrite(INA_M1, LOW);
    digitalWrite(INB_M1, LOW);
  }

  // 4. Controle Direcional - Esteira Direita
  if (v_right > 0) { // Frente
    digitalWrite(INA_M2, HIGH);
    digitalWrite(INB_M2, LOW);
  } else if (v_right < 0) { // Trás
    digitalWrite(INA_M2, LOW);
    digitalWrite(INB_M2, HIGH);
  } else { // Parado
    digitalWrite(INA_M2, LOW);
    digitalWrite(INB_M2, LOW);
  }

  // 5. Enviar PWM 
  ledcWrite(EN_M1, pwm_left);
  ledcWrite(EN_M2, pwm_right);
  
  digitalWrite(LED_PIN, (v == 0 && w == 0) ? LOW : HIGH);
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(INA_M1, OUTPUT);
  pinMode(INB_M1, OUTPUT);
  pinMode(INA_M2, OUTPUT);
  pinMode(INB_M2, OUTPUT);

  digitalWrite(INA_M1, LOW);
  digitalWrite(INB_M1, LOW);
  digitalWrite(INA_M2, LOW);
  digitalWrite(INB_M2, LOW);

  // Inicializar PWM em 11 bits (0 a 2047)
  ledcAttach(EN_M1, PWM_FREQ, PWM_RES);
  ledcAttach(EN_M2, PWM_FREQ, PWM_RES);
  
  ledcWrite(EN_M1, 0);
  ledcWrite(EN_M2, 0);

  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
