#include <Wire.h>
#include <TMCStepper.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>

// --- CONFIGURAÇÕES DO ENCODER MT6701 (I2C) ---
#define SDA_PIN 13
#define SCL_PIN 14
#define MT6701_I2C_ADDR 0x06 
#define MT6701_ANGLE_REG 0x03

// --- CONFIGURAÇÕES DOS MOTORES TMC2209 (UART) ---
#define RX_PIN 18 
#define TX_PIN 17 
#define SERIAL_PORT Serial1 
#define R_SENSE 0.11f        

#define ENDERECO_MOTOR_0 0b00 
#define ENDERECO_MOTOR_1 0b01 
#define ENDERECO_MOTOR_2 0b10 
#define ENDERECO_MOTOR_3 0b11 

// --- CONFIGURAÇÕES DO LATCH ---
#define LATCH_PIN 21          // Vai HIGH assim que a ESP32 liga
#define LATCH_TRIGGER_PIN 6   // Se ficar HIGH por 3s contínuos, desliga o LATCH_PIN
#define LATCH_HOLD_TIME_MS 3000UL

bool latchAtivo = true;              // Estado atual do LATCH_PIN
bool triggerContando = false;        // Se já estamos contando o tempo do trigger em HIGH
unsigned long triggerHighDesde = 0;  // Timestamp de quando o trigger foi para HIGH

TMC2209Stepper driver0(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_0);
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_3);

// --- VARIÁVEIS DO MICRO-ROS ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Publisher e Subscribers
rcl_publisher_t pub_encoder;
rcl_subscription_t sub_joint1;
rcl_subscription_t sub_joint2;
rcl_subscription_t sub_height;
rcl_subscription_t sub_claw;

// Mensagens
std_msgs__msg__Float64 msg_encoder;
std_msgs__msg__Float64 msg_joint1;
std_msgs__msg__Float64 msg_joint2;
std_msgs__msg__Float64 msg_height;
std_msgs__msg__Float64 msg_claw;

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- CALLBACKS DOS MOTORES ---
void callback_joint1(const void * msgin) {
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  driver0.VACTUAL((long)msg->data); 
}

void callback_joint2(const void * msgin) {
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  driver1.VACTUAL((long)msg->data);
}

void callback_height(const void * msgin) {
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  driver2.VACTUAL((long)msg->data);
}

void callback_claw(const void * msgin) {
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  driver3.VACTUAL((long)msg->data);
}

// --- LÓGICA DO LATCH ---
void atualizarLatch() {
  if (!latchAtivo) {
    // Já desligado, nada a fazer
    return;
  }

  bool triggerHigh = (digitalRead(LATCH_TRIGGER_PIN) == HIGH);

  if (triggerHigh) {
    if (!triggerContando) {
      // Começou a subida agora, inicia a contagem
      triggerContando = true;
      triggerHighDesde = millis();
    } else if (millis() - triggerHighDesde >= LATCH_HOLD_TIME_MS) {
      // Ficou HIGH pelo tempo necessário: desliga o latch
      digitalWrite(LATCH_PIN, LOW);
      latchAtivo = false;
      triggerContando = false;
    }
  } else {
    // Caiu para LOW antes de completar 3s: reseta a contagem
    triggerContando = false;
  }
}

void setup() {
  // --- LATCH: liga GPIO21 assim que a ESP32 inicializa ---
  pinMode(LATCH_PIN, OUTPUT);
  digitalWrite(LATCH_PIN, HIGH);
  pinMode(LATCH_TRIGGER_PIN, INPUT); // ajuste para INPUT_PULLDOWN/INPUT_PULLUP se necessário no seu circuito

  // Inicializa o transporte do micro-ROS
  set_microros_transports();
  
  // Inicia o barramento I2C (Encoder)
  Wire.begin(SDA_PIN, SCL_PIN); 

  // Inicia o barramento UART (Motores)
  SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inicialização e Configuração dos TMC2209
  driver0.begin(); driver1.begin(); driver2.begin(); driver3.begin();
  driver0.pdn_disable(1); driver1.pdn_disable(1); driver2.pdn_disable(1); driver3.pdn_disable(1);
  driver0.toff(5); driver1.toff(5); driver2.toff(5); driver3.toff(5); 
  driver0.rms_current(1000); driver1.rms_current(1000); driver2.rms_current(1000); driver3.rms_current(1000); 
  driver0.microsteps(16); driver1.microsteps(16); driver2.microsteps(16); driver3.microsteps(16);
  driver0.en_spreadCycle(false); driver1.en_spreadCycle(false); driver2.en_spreadCycle(false); driver3.en_spreadCycle(false); 
  driver0.pwm_autoscale(true); driver1.pwm_autoscale(true); driver2.pwm_autoscale(true); driver3.pwm_autoscale(true);   

  driver0.TCOOLTHRS(0xFFFFF); driver1.TCOOLTHRS(0xFFFFF); driver2.TCOOLTHRS(0xFFFFF); driver3.TCOOLTHRS(0xFFFFF); 
  driver0.SGTHRS(50); driver1.SGTHRS(50); driver2.SGTHRS(50); driver3.SGTHRS(50); 
  driver0.semin(5); driver1.semin(5); driver2.semin(5); driver3.semin(5);
  driver0.semax(2); driver1.semax(2); driver2.semax(2); driver3.semax(2); 
  driver0.sedn(0b01); driver1.sedn(0b01); driver2.sedn(0b01); driver3.sedn(0b01); 
  driver0.seimin(0); driver1.seimin(0); driver2.seimin(0); driver3.seimin(0); 
  driver0.seup(0b11); driver1.seup(0b11); driver2.seup(0b11); driver3.seup(0b11); 

  delay(2000); // Aguarda estabilização dos barramentos

  allocator = rcl_get_default_allocator();

  // --- LOOP DE TENTATIVA DE CONEXÃO (À prova de travamento) ---
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    delay(500); 
  }
  
  // Criação do nó unificado
  rclc_node_init_default(&node, "scara_steppers_and_encoder_node", "", &support);

  // --- INICIALIZAÇÃO DO PUBLISHER (Encoder) ---
  rclc_publisher_init_default(
    &pub_encoder, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/encoder/arm1");

  // --- INICIALIZAÇÃO DOS SUBSCRIBERS (Motores) ---
  rclc_subscription_init_default(
    &sub_joint1, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/scara_arm_joint1/cmd_vel");

  rclc_subscription_init_default(
    &sub_joint2, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/scara_arm_joint2/cmd_vel");

  rclc_subscription_init_default(
    &sub_height, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/scara_height_jointl/cmd_vel");

  rclc_subscription_init_default(
    &sub_claw, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/scara_claw_rotation_jointl/cmd_vel");

  // --- CONFIGURAÇÃO DO EXECUTOR ---
  // O número 4 indica a quantidade de handles (4 subscribers). Publishers não entram na contagem do executor.
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &sub_joint1, &msg_joint1, &callback_joint1, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_joint2, &msg_joint2, &callback_joint2, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_height, &msg_height, &callback_height, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_claw, &msg_claw, &callback_claw, ON_NEW_DATA);
}

void loop() {
  // --- LATCH: verifica se o trigger (GPIO6) pede para desligar o GPIO21 ---
  atualizarLatch();

  // --- PARTE 1: LEITURA E PUBLICAÇÃO DO ENCODER ---
  Wire.beginTransmission(MT6701_I2C_ADDR);
  Wire.write(MT6701_ANGLE_REG);
  Wire.endTransmission(false); 

  Wire.requestFrom((uint16_t)MT6701_I2C_ADDR, (uint8_t)2); 

  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read(); 
    uint8_t lsb = Wire.read(); 

    uint16_t angleRaw = (msb << 6) | (lsb >> 2);
    double angleDegrees = (double)angleRaw * 360.0 / 16384.0;

    msg_encoder.data = angleDegrees;
    RCSOFTCHECK(rcl_publish(&pub_encoder, &msg_encoder, NULL));
  }

  // --- PARTE 2: EXECUÇÃO DOS CALLBACKS DOS MOTORES ---
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  // Atraso unificado para manter a estabilidade do loop
  delay(10);
}
