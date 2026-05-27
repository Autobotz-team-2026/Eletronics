#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Definição dos pinos I2C personalizados para a ESP32-S3
#define SDA_PIN 8
#define SCL_PIN 17

// Variáveis globais para o cálculo do Yaw
float yaw = 0.0;
unsigned long tempoAnterior = 0;

// --- VARIÁVEL PARA O OFFSET AUTOMÁTICO ---
// Usando 'double' no ESP32 para garantir altíssima precisão nas casas decimais
double gyroZ_offset = 0.0; 

float kalman(float input) {
  static float Q = 0.05;      // Ajustado para responder bem a movimentos rápidos
  static float R = 10.0;      // Reduzido para evitar o "atraso" de não chegar aos 90º
  static float x_est = 0.0;  // estado estimado atual
  static float P = 1.0;      // estimativa inicial de erro

  float z = (float)input;    
  float x_pred = x_est;      
  float P_pred = P + Q;      
  float K = P_pred / (P_pred + R);  

  x_est = x_pred + K * (z - x_pred); 
  P = (1 - K) * P_pred;              

  return x_est;             
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
  
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  
  // --- ROTINA DE AUTOCALIBRAÇÃO EXPANDIDA (2000 AMOSTRAS) ---
  Serial.println("AQUECENDO E CALIBRANDO... NAO MEXA NA PLACA!");
  
  double soma = 0.0; // Usando DOUBLE para não perder o final das casas decimais na soma
  int amostrasDescarte = 500; // Descarta leituras iniciais (aquecimento do chip)
  int totalAmostras = 2000;    // 2000 amostras para precisão estatística (~10 segundos)
  
  // 1. Descarta leituras iniciais de aquecimento
  for (int i = 0; i < amostrasDescarte; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    delay(5);
  }

  // 2. Coleta os dados BRUTOS (sem Kalman) para achar o centro real
  for (int i = 0; i < totalAmostras; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Variável temporária também em double
    double gyroZ_deg = g.gyro.z * (180.0 / PI);
    soma += gyroZ_deg; 
    delay(5); 
  }
  
  // 3. Média exata do erro do giroscópio + SEU AJUSTE FINO TÉRMICO
  gyroZ_offset = (soma / totalAmostras) + 0.004455;
  
  Serial.print("Calibracao Concluida! Novo Offset (com ajuste): ");
  Serial.println(gyroZ_offset, 6);
  
  // Inicializa o tempo APÓS a calibração
  tempoAnterior = micros();
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 1. Calcula o delta T (tempo decorrido em segundos)
  unsigned long tempoAtual = micros();
  float dt = (tempoAtual - tempoAnterior) / 1000000.0; 
  tempoAnterior = tempoAtual;

  // 2. Aplica o filtro de Kalman no dado bruto do giroscópio
  float filtrado = kalman(g.gyro.z);
  
  // 3. Converte para graus/s e subtrai o Offset calibrado
  float zfiltrado = (filtrado * (180.0 / PI)) - gyroZ_offset;

  // 4. Integração para encontrar o Yaw em graus
  yaw = yaw + (zfiltrado * dt);

  // --- FORMATO PARA O SERIAL PLOTTER ---
  // Serial.print("Vel_Z_Filtrada:");
  // Serial.println(zfiltrado, 4); 

  // Se quiser testar o ângulo direto, mude o Plotter para ler esta linha:
  Serial.print("Yaw_Angulo:");
  Serial.println(yaw, 2); 
}
