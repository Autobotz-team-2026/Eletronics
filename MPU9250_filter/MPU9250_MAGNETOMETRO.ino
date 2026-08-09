#include <Wire.h>
#include <math.h>

// Pinos e Endereços I2C do MPU9250
#define SDA_PIN 8
#define SCL_PIN 46
#define MPU_ADDR 0x68
#define MAG_ADDR 0x0C

// Registradores
#define PWR_MGMT_1   0x6B
#define INT_PIN_CFG  0x37
#define ACCEL_XOUT_H 0x3B
#define MAG_CNTL1    0x0A
#define MAG_HXL      0x03

// =======================================================
// AJUSTES MANUAIS DE CALIBRAÇÃO (Magnetômetro)
// =======================================================
// Valores extraídos do seu código de calibração
float mag_bias_x = -118.5000;
float mag_bias_y = -183.0000;
float mag_bias_z = 915.0000;

float mag_scale_x = 1.1035;
float mag_scale_y = 1.0189;
float mag_scale_z = 0.8990;
// =======================================================

// --- Filtro de Kalman adaptado para Ângulos (0 a 360) ---
float kalmanAngulo(float z) {
  static float Q = 0.05;     // Variância do processo
  static float R = 10.0;     // Variância da medição (Aumente para filtrar mais o ruído)
  static float x_est = 0.0;  // Estado estimado atual
  static float P = 1.0;      // Estimativa inicial de erro

  float x_pred = x_est;      // Estado de predição
  float P_pred = P + Q;      // Atualiza erro de predição
  float K = P_pred / (P_pred + R); // Calcula o ganho de Kalman

  // Calcula o erro (diferença entre medição e predição)
  float erro = z - x_pred;

  // CORREÇÃO CIRCULAR: Garante que o erro vá pelo caminho mais curto
  if (erro > 180.0) erro -= 360.0;
  if (erro < -180.0) erro += 360.0;

  // Atualiza a estimativa com base no erro corrigido
  x_est = x_pred + K * erro;
  P = (1 - K) * P_pred;      // Atualiza o erro

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
  delay(100);

  Serial.println("\n--- Iniciando Bússola Compensada (Sem Biblioteca) ---");

  // 1. Acorda MPU6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  // 2. Ativa Bypass Mode (Ponte direta para o Magnetômetro AK8963)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(50);

  // 3. Configura o Magnetômetro (Contínuo 100Hz, 16-bits)
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MAG_CNTL1);
  Wire.write(0x16);
  Wire.endTransmission();
  delay(50);

  Serial.println("Sensor conectado com sucesso!");
  Serial.println("A utilizar valores de calibração magnética fixos. A iniciar leituras...");
  delay(1000);
}

void loop() {
  // --- 1. LER ACELERÔMETRO ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6);
  float ax = (int16_t)(Wire.read() << 8 | Wire.read());
  float ay = (int16_t)(Wire.read() << 8 | Wire.read());
  float az = (int16_t)(Wire.read() << 8 | Wire.read());

  // --- 2. LER MAGNETÔMETRO ---
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MAG_HXL);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, 7);
  
  if (Wire.available() >= 7) {
    int16_t raw_x = Wire.read() | (Wire.read() << 8);
    int16_t raw_y = Wire.read() | (Wire.read() << 8);
    int16_t raw_z = Wire.read() | (Wire.read() << 8);
    Wire.read(); // Lê o ST2 para liberar a próxima medição

    // Aplica a calibração com os seus valores
    float cal_x = (raw_x - mag_bias_x) * mag_scale_x;
    float cal_y = (raw_y - mag_bias_y) * mag_scale_y;
    float cal_z = (raw_z - mag_bias_z) * mag_scale_z;

    // ALINHAMENTO DE EIXOS (Específico do hardware MPU9250)
    // O magnetômetro interno é rotacionado fisicamente em relação ao acelerômetro
    float mx = cal_y;
    float my = cal_x;
    float mz = -cal_z;

    // --- 3. CÁLCULO DE INCLINAÇÃO (Roll e Pitch) ---
    float roll  = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // --- 4. COMPENSAÇÃO DE INCLINAÇÃO (Tilt Compensation) ---
    float X_comp = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
    float Y_comp = my * cos(roll) - mz * sin(roll);

    // --- 5. YAW BRUTO ---
    float yaw_bruto = atan2(Y_comp, X_comp) * 180.0 / PI;
    if (yaw_bruto < 0.0) {
      yaw_bruto += 360.0;
    }

    // --- 6. FILTRO DE KALMAN ---
    float yaw_filtrado = kalmanAngulo(yaw_bruto);

    // --- FORMATO PARA O SERIAL PLOTTER ---
    Serial.print("Yaw_Bruto:");
    Serial.print(yaw_bruto);
    Serial.print(",");
    Serial.print("Yaw_Filtrado_Kalman:");
    Serial.println(yaw_filtrado);
  }
  
  delay(50); // Delay mantido em 50ms igual ao seu código de referência
}
