#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



/*===================================================
este codigo aplica uma maneira semelhante a deadzone
nos testes realizados, mesmo com esse deadzone de 0.8
a mpu respondeu bem, variações rápidas de 0 a 90 não 
houve perda de ângulo
=====================================================*/

/*====================================================
dos 3 códigos neste repositório para a mpu6050, este é
o que apresenta uma maior estabilidade do valor quando 
a mpu é deixada parada. Porém, é necessário testar e 
ver se a funcionalidade adicionada para filtragem afeta
negativamente o movimento do robô
======================================================*/


Adafruit_MPU6050 mpu;

// Definição dos pinos I2C personalizados para a ESP32-S3
#define SDA_PIN 8
#define SCL_PIN 17

// Variáveis globais para o cálculo do Yaw AGORA EM DOUBLE
double yaw = 0.0;
unsigned long tempoAnterior = 0;

// --- VARIÁVEL PARA O OFFSET AUTOMÁTICO E DINÂMICO ---
double gyroZ_offset = 0.0; 

// Filtro de Kalman agora trabalha 100% com 64 bits de precisão
double kalman(double input) {
  static double Q = 0.05;      
  static double R = 10.0;      
  static double x_est = 0.0;  
  static double P = 1.0;      

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
  
  // --- ROTINA DE AQUECIMENTO (30 SEGUNDOS) E CALIBRAÇÃO (15 SEGUNDOS) ---
  Serial.println("INICIANDO AQUECIMENTO (30 segundos)...");
  
  double soma = 0.0; 
  unsigned long amostrasDescarte = 6000; // 30 segundos (6000 * 5ms)
  unsigned long totalAmostras = 3000;    // 15 segundos (3000 * 5ms)
  
  // 1. Fase de Aquecimento
  for (unsigned long i = 0; i < amostrasDescarte; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // A cada 2000 leituras (10 segundos), avisa no monitor serial
    if (i % 2000 == 0) {
        Serial.print("Aquecendo... Temp atual: ");
        Serial.print(temp.temperature, 2);
        Serial.println(" C");
    }
    delay(5);
  }

  Serial.println("AQUECIMENTO CONCLUIDO. INICIANDO CALIBRACAO DE 15s (NAO MEXA NA PLACA!)");

  // 2. Fase de Calibração a Quente
  for (unsigned long i = 0; i < totalAmostras; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    double gyroZ_deg = (double)g.gyro.z * (180.0 / M_PI);
    soma += gyroZ_deg; 
    
    // A cada 1000 leituras (5 segundos), avisa o progresso
    if (i > 0 && i % 1000 == 0) {
        Serial.println("Calibrando... (mais 5 segundos concluidos)");
    }
    delay(5); 
  }
  
  // 3. Média exata
  gyroZ_offset = (soma / (double)totalAmostras); 
  
  Serial.print("Calibracao Concluida! Offset Inicial: ");
  Serial.println(gyroZ_offset, 6);
  
  // Inicializa o tempo APÓS a calibração
  tempoAnterior = micros();
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 

  // 1. Calcula o delta T
  unsigned long tempoAtual = micros();
  double dt = (double)(tempoAtual - tempoAnterior) / 1000000.0; 
  tempoAnterior = tempoAtual;

  // 2. Aplica o filtro de Kalman
  double filtrado = kalman((double)g.gyro.z);
  
  // 3. Converte para graus/s (Leitura Base)
  double z_graus_s = filtrado * (180.0 / M_PI);

  // --- RASTREAMENTO DINÂMICO DE OFFSET ---
  // Corrige os picos assimétricos e o drift constante
  if (abs(z_graus_s - gyroZ_offset) < 0.8) {
      gyroZ_offset = (gyroZ_offset * 0.999) + (z_graus_s * 0.001);
  }

  // 4. Subtrai o offset dinâmico
  double zfiltrado = z_graus_s - gyroZ_offset;

  // 5. Integração
  yaw = yaw + (zfiltrado * dt);

  // --- FORMATO PARA O SERIAL PLOTTER E MONITOR ---
  Serial.print("Yaw_Angulo:");
  Serial.print(yaw, 2); 
  
  Serial.print(", Temp_C:");
  Serial.println(temp.temperature, 2); 
}
