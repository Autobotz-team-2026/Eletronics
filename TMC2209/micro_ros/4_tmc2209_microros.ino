#include <TMCStepper.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>

// Pinos e configurações do ESP32-S3
#define RX_PIN 18 
#define TX_PIN 17 
#define SERIAL_PORT Serial1 

#define R_SENSE 0.11f        

// --- ENDEREÇOS DOS DRIVERS ---
#define ENDERECO_MOTOR_0 0b00 
#define ENDERECO_MOTOR_1 0b01 
#define ENDERECO_MOTOR_2 0b10 
#define ENDERECO_MOTOR_3 0b11 

TMC2209Stepper driver0(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_0);
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_3);

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t sub_joint1;
rcl_subscription_t sub_joint2;
rcl_subscription_t sub_height;
rcl_subscription_t sub_claw;

std_msgs__msg__Float64 msg_joint1;
std_msgs__msg__Float64 msg_joint2;
std_msgs__msg__Float64 msg_height;
std_msgs__msg__Float64 msg_claw;

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

void setup() {
  set_microros_transports();
  
  SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inicialização dos TMC2209
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

  delay(1000);

  allocator = rcl_get_default_allocator();

  // --- LOOP DE TENTATIVA DE CONEXÃO (Não trava mais a placa) ---
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    delay(500); // Tenta reconectar a cada meio segundo até o Agent aceitar
  }
  
  rclc_node_init_default(&node, "scara_steppers_node", "", &support);

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

  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_subscription(&executor, &sub_joint1, &msg_joint1, &callback_joint1, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_joint2, &msg_joint2, &callback_joint2, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_height, &msg_height, &callback_height, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_claw, &msg_claw, &callback_claw, ON_NEW_DATA);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(5);
}
