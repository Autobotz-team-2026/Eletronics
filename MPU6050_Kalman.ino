#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;


/*==========================================================================
aumentar as casas da adição no calculo de zfiltrado pode aumentar a precisão 
do ângulo final
============================================================================*/




// Definição dos pinos I2C personalizados para a ESP32-S3
#define SDA_PIN 8
#define SCL_PIN 17

// Variáveis globais para o cálculo do Yaw
float yaw = 0.0;
unsigned long tempoAnterior = 0;

float kalman(float input) {
  static float Q = 0.05;     // variância do processo
  static float R = 10.0;      // variância da medição
  static float x_est = 0.0;  // estado estimado atual
  static float P = 1.0;      // estimativa inicial de erro

  float z = (float)input;    // armazena a medida atual
  float x_pred = x_est;      // estado de predição
  float P_pred = P + Q;      // atualiza erro de predição
  float K = P_pred / (P_pred + R);  // calcula o ganho de Kalman

  x_est = x_pred + K * (z - x_pred); // atualiza a estimativa
  P = (1 - K) * P_pred;              // atualiza o erro

  return x_est;             // retorna o estado atual "filtrado"
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); 
  }

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Erro:MPU6050_nao_encontrado");
    while (1) {
      delay(10); 
    }
  }
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  // Inicializa o tempo anterior
  tempoAnterior = millis();
  
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 1. Calcula o delta T (tempo decorrido em segundos)
  unsigned long tempoAtual = millis();
  float dt = (tempoAtual - tempoAnterior) / 1000.0; // Divide por 1000 para converter ms em segundos
  tempoAnterior = tempoAtual;

  // 2. Aplica o filtro de Kalman e converte para graus/s
  float filtrado = kalman(g.gyro.z);
  float gyroZ_deg = g.gyro.z * (180.0 / PI);
  float zfiltrado = (filtrado * (180.0 / PI)) + 1.40;

  // 3. Integração para encontrar o Yaw em graus
  // Ignora ruídos muito pequenos perto do zero (opcional, mas recomendado)
    yaw = yaw + (zfiltrado * dt);
 

  // --- FORMATO PARA O SERIAL PLOTTER ---
  /*Serial.print("Vel_Z_Filtrada:");
  Serial.println(zfiltrado, 2);
  *///Serial.print(",");

  Serial.print("Yaw_Angulo:");
  Serial.println(yaw, 2); 
}
