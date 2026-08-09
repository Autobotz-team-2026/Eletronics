/*=============================================
Observação: Esse codigo está pinado com o led 
rgb da ESP32S3-N16R8, a utilizada no laboratorio
é diferente.
===============================================*/


#include <WiFi.h> 
#include <SPI.h>  
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 48 // O pino do seu LED RGB
#define NUMPIXELS 1 // Quantidade de LEDs (1 por ser o interno)

// Inicializa o objeto do LED RGB
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

rcl_subscription_t subscriber;
std_msgs__msg__Bool msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    // Pisca em VERMELHO se der erro no micro-ROS
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); 
    pixels.show();
    delay(100);
    pixels.clear();
    pixels.show();
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  
  if (msg->data == true) {
    // Liga o LED RGB na cor VERDE
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); 
  } else {
    // Desliga o LED RGB
    pixels.clear(); 
  }
  pixels.show(); // Aplica a cor no LED
}

void setup() {
  set_microros_transports();
  
  // Inicializa o LED RGB e garante que ele comece apagado
  pixels.begin();
  pixels.clear();
  pixels.show();
  
  delay(2000); 

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_led_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/led_command"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
