#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/time_reference.h>

// ============================================
// CONFIGURAÇÕES
// ============================================
#define SDA_PIN 8
#define SCL_PIN 46
#define PUBLISH_FREQUENCY_HZ 50
#define PUBLISH_INTERVAL_MS (1000 / PUBLISH_FREQUENCY_HZ)

// ============================================
// OBJETOS GLOBAIS
// ============================================
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;

// ============================================
// VARIÁVEIS MICRO-ROS
// ============================================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t imu_publisher;
rcl_publisher_t time_publisher;
rcl_timer_t timer;
rclc_executor_t executor;

sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__TimeReference time_msg;

bool micro_ros_ready = false;

// ============================================
// CALIBRAÇÃO
// ============================================
void calibrateMPU() {
    Serial.println("Calibrando MPU6050. Mantenha o sensor PARADO!");
    delay(2000);

    mpu.CalibrateGyro(6);
    mpu.CalibrateAccel(6);

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    Serial.println("Calibração concluída!");
}

// ============================================
// LEITURA DO SENSOR
// ============================================
void readIMU() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    accel_x = (ax / 16384.0) * 9.80665;
    accel_y = (ay / 16384.0) * 9.80665;
    accel_z = (az / 16384.0) * 9.80665;

    gyro_x = (gx / 131.0) * 0.0174533;
    gyro_y = (gy / 131.0) * 0.0174533;
    gyro_z = (gz / 131.0) * 0.0174533;
}

// ============================================
// PUBLICAÇÃO
// ============================================
void publishIMU() {
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = 8;
    imu_msg.header.frame_id.capacity = 8;

    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 1.0;

    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0;
        imu_msg.angular_velocity_covariance[i] = 0;
        imu_msg.linear_acceleration_covariance[i] = 0;
    }

    imu_msg.orientation_covariance[0] = 0.1;
    imu_msg.orientation_covariance[4] = 0.1;
    imu_msg.orientation_covariance[8] = 0.1;

    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;

    imu_msg.linear_acceleration_covariance[0] = 0.001;
    imu_msg.linear_acceleration_covariance[4] = 0.001;
    imu_msg.linear_acceleration_covariance[8] = 0.001;

    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;

    imu_msg.linear_acceleration.x = accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = accel_z;

    rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void publishTime() {
    time_msg.header.stamp.sec = millis() / 1000;
    time_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    time_msg.time_ref.sec = millis() / 1000;
    time_msg.time_ref.nanosec = (millis() % 1000) * 1000000;
    time_msg.source.data = (char*)"esp32_system_time";
    time_msg.source.size = 17;
    time_msg.source.capacity = 17;

    rcl_publish(&time_publisher, &time_msg, NULL);
}

void timer_callback(rcl_timer_t *t, int64_t last_call_time) {
    (void)last_call_time;
    if (t != NULL) {
        readIMU();
        publishIMU();
        publishTime();

        static int count = 0;
        if (count++ >= 50) {
            Serial.printf("Accel: %.2f %.2f %.2f | Gyro: %.3f %.3f %.3f\n",
                          accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
            count = 0;
        }
    }
}

// ============================================
// SETUP
// ============================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    Serial.println("Inicializando MPU6050...");
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("ERRO: MPU6050 não encontrado!");
        Serial.println("Verifique as conexões!");
        while(1) delay(100);
    }

    Serial.println("MPU6050 conectado!");
    calibrateMPU();

    // Inicializa micro-ROS
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "imu_node", "", &support);

    rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                    "imu/data_raw"
    );

    rclc_publisher_init_best_effort(
        &time_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, TimeReference),
                                    "time_reference"
    );

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(PUBLISH_INTERVAL_MS),
                            timer_callback
    );

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    micro_ros_ready = true;

    Serial.printf("micro-ROS pronto! Publicando a %d Hz\n", PUBLISH_FREQUENCY_HZ);
    Serial.println("Tópicos: /imu/data_raw e /time_reference");
}

// ============================================
// LOOP
// ============================================
void loop() {
    if (micro_ros_ready) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    delay(5);
}