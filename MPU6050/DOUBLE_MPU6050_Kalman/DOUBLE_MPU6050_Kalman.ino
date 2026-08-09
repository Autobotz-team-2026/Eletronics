#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

/*==========================================================================
Este código apresentou uma resposta melhor com o decorrer do tempo em relação
ao código com float - Atualizado para padronizar as unidades de medida (Graus/s)
============================================================================*/

// Definição dos pinos I2C personalizados para a ESP32-S3
#define SDA_PIN 8
#define SCL_PIN 17

// Variáveis globais para o cálculo do Yaw AGORA EM DOUBLE
double yaw = 0.0;
unsigned long tempoAnterior = 0;

// --- VARIÁVEL PARA O OFFSET AUTOMÁTICO ---
// Usando 'double' no ESP32 para garantir altíssima precisão nas casas decimais
double gyroZ_offset = 0.0; 

// Filtro de Kalman agora trabalha 100% com 64 bits de precisão
double kalman(double input) {
  static double Q = 0.5;      // Ajustado (era 0.05) para responder bem a movimentos rápidos
  static double R = 10.0;      // Reduzido (era 10.0) para evitar o "atraso" de não chegar aos 90º
  static double x_est = 0.0;  // estado estimado atual
  static double P = 1.0;      // estimativa inicial de erro

  double z = input;    
  double x_pred = x_est;      
  double P_pred = P + Q;      
  double K = P_pred / (P_pred + R);  

  x_est = x_pred + K * (z - x_pred); 
  P = (1.0 - K) * P_pred;              

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
  
  double soma = 0.0; 
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
    
    // Variável em double, usando a constante M_PI de alta precisão
    double gyroZ_deg = (double)g.gyro.z * (180.0 / M_PI);
    soma += gyroZ_deg; 
    delay(5); 
  }
  
  // 3. Média exata do erro do giroscópio + SEU AJUSTE FINO TÉRMICO
  gyroZ_offset = (soma / (double)totalAmostras);
  
  Serial.print("Calibracao Concluida! Novo Offset (com ajuste): ");
  Serial.println(gyroZ_offset, 6);
  
  // Inicializa o tempo APÓS a calibração
  tempoAnterior = micros();
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 1. Calcula o delta T em DOUBLE
  unsigned long tempoAtual = micros();
  double dt = (double)(tempoAtual - tempoAnterior) / 1000000.0; 
  tempoAnterior = tempoAtual;

  // 2. Converte a leitura bruta (rad/s) para graus/s PRIMEIRO
  double gyroZ_bruto_deg = (double)g.gyro.z * (180.0 / M_PI);

  // 3. Subtrai o offset (agora ambos estão na mesma escala: graus/s)
  double gyroz_corrigido = gyroZ_bruto_deg - gyroZ_offset;

  // 4. Aplica o filtro de Kalman 
  double zfiltrado = kalman(gyroz_corrigido);

  // 5. Integração de alta precisão para encontrar o Yaw em graus
  yaw = yaw + (zfiltrado * dt);
  
  // Imprime o bruto em GRAUS para você poder comparar lado a lado com o corrigido
  Serial.print("YAW:");
  Serial.println(yaw, 2); 
  
  // O valor corrigido agora deve oscilar muito próximo a zero quando a placa estiver parada
  //Serial.print("Corrigido_Deg:");
 // Serial.println(gyroz_corrigido, 2);
  
  delay(10);
}
