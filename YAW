#include <Wire.h>
#include "ICM_20948.h"

#define SDA_PIN 8 
#define SCL_PIN 46
#define AD0_VAL 0 

ICM_20948_I2C myICM;

// Valores de calibração magnética obtidos através do codigo de calibragem
float mag_bias_x = 12.6700;
float mag_bias_y = 30.8000;
float mag_bias_z = 107.5500;

float mag_scale_x = 1.4966;
float mag_scale_y = 0.7364;
float mag_scale_z = 1.0269;

// --- Filtro de Kalman adaptado para Ângulos (0 a 360) ---
float kalmanAngulo(float z) {
  static float Q = 0.05;     // variância do processo
  static float R = 10.0;     // variância da medição (aumente para filtrar mais o ruído)
  static float x_est = 0.0;  // estado estimado atual
  static float P = 1.0;      // estimativa inicial de erro

  float x_pred = x_est;      // estado de predição
  float P_pred = P + Q;      // atualiza erro de predição
  float K = P_pred / (P_pred + R);  // calcula o ganho de Kalman

  // Calcula o erro (diferença entre medição e predição)
  float erro = z - x_pred;

  // CORREÇÃO CIRCULAR: Garante que o erro vá pelo caminho mais curto (evita o problema dos 360º)
  if (erro > 180.0) erro -= 360.0;
  if (erro < -180.0) erro += 360.0;

  // Atualiza a estimativa com base no erro corrigido
  x_est = x_pred + K * erro;
  P = (1 - K) * P_pred;              // atualiza o erro

  // Mantém a saída filtrada dentro do limite de 0 a 360 graus
  if (x_est < 0.0) x_est += 360.0;
  if (x_est >= 360.0) x_est -= 360.0;

  return x_est;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 

  bool connected = false;
  while (!connected) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("A procurar o sensor no I2C...");
      delay(500);
    } else {
      connected = true;
    }
  }
  
  Serial.println("Sensor conectado com sucesso!");
  Serial.println("A utilizar valores de calibração magnética fixos. A iniciar leituras...");
  delay(1000); // Pausa breve apenas para leitura no terminal antes de arrancar
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT(); 

    float ax = myICM.accX();
    float ay = myICM.accY();
    float az = myICM.accZ();

    // Aplica os valores de calibração fixos diretamente nas leituras do magnetómetro
    float mx = (myICM.magX() - mag_bias_x) * mag_scale_x;
    float my = (myICM.magY() - mag_bias_y) * mag_scale_y;
    float mz = (myICM.magZ() - mag_bias_z) * mag_scale_z;

    // Calcula o Roll e o Pitch em radianos para a compensação
    float roll  = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // Fórmula Padrão de Compensação de Inclinação
    float X_comp = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
    float Y_comp = my * cos(roll) - mz * sin(roll);

    // Yaw bruto (ainda com o ruído natural do ambiente)
    float yaw_bruto = atan2(Y_comp, X_comp) * 180.0 / PI;
    if (yaw_bruto < 0) {
      yaw_bruto += 360.0;
    }

    // Aplica o filtro de Kalman no Yaw bruto
    float yaw_filtrado = kalmanAngulo(yaw_bruto);

    // --- FORMATO PARA O SERIAL PLOTTER ---
    Serial.print("Yaw_Bruto:");
    Serial.print(yaw_bruto);
    Serial.print(",");
    Serial.print("Yaw_Filtrado_Kalman:");
    Serial.println(yaw_filtrado);
    
    delay(50);
  }
}
