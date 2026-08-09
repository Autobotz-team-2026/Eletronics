#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"

// ============================================
// CONFIGURAÇÕES DOS PINOS
// ============================================
#define SDA_PIN 8 
#define SCL_PIN 46
#define AD0_VAL 0  // 0 = 0x69, 1 = 0x68

// ============================================
// OBJETO GLOBAL
// ============================================
ICM_20948_I2C myICM;

// ============================================
// VARIÁVEIS DE CALIBRAÇÃO
// ============================================
float mag_bias_x = 0, mag_bias_y = 0, mag_bias_z = 0;
float mag_scale_x = 1.0, mag_scale_y = 1.0, mag_scale_z = 1.0;
bool calibrated = false;

// ============================================
// FUNÇÃO PARA CALIBRAÇÃO
// ============================================
void calibrateMagnetometer() {
  Serial.println("\n==========================================");
  Serial.println("   CALIBRAÇÃO DO MAGNETÔMETRO");
  Serial.println("==========================================");
  Serial.println("ATENÇÃO: Gire o sensor em todas as direções");
  Serial.println("em formato de '8' por 15 segundos!");
  Serial.println("Iniciando em 5 segundos...\n");
  delay(5000);
  
  Serial.println("COLETANDO DADOS... GIRE O SENSOR AGORA!\n");
  
  float mag_min[3] = {32767, 32767, 32767};
  float mag_max[3] = {-32768, -32768, -32768};
  
  unsigned long start = millis();
  int samples = 0;
  
  while(millis() - start < 15000) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      
      float mx = myICM.magX();
      float my = myICM.magY();
      float mz = myICM.magZ();
      
      // Atualiza mínimos
      if (mx < mag_min[0]) mag_min[0] = mx;
      if (my < mag_min[1]) mag_min[1] = my;
      if (mz < mag_min[2]) mag_min[2] = mz;
      
      // Atualiza máximos
      if (mx > mag_max[0]) mag_max[0] = mx;
      if (my > mag_max[1]) mag_max[1] = my;
      if (mz > mag_max[2]) mag_max[2] = mz;
      
      samples++;
      if (samples % 100 == 0) Serial.print(".");
    }
    delay(5);
  }
  
  // Calcula Hard Iron (bias)
  mag_bias_x = (mag_max[0] + mag_min[0]) / 2.0;
  mag_bias_y = (mag_max[1] + mag_min[1]) / 2.0;
  mag_bias_z = (mag_max[2] + mag_min[2]) / 2.0;
  
  // Calcula Soft Iron (scale)
  float range_x = (mag_max[0] - mag_min[0]) / 2.0;
  float range_y = (mag_max[1] - mag_min[1]) / 2.0;
  float range_z = (mag_max[2] - mag_min[2]) / 2.0;
  float avg_range = (range_x + range_y + range_z) / 3.0;
  
  mag_scale_x = avg_range / range_x;
  mag_scale_y = avg_range / range_y;
  mag_scale_z = avg_range / range_z;
  
  calibrated = true;
  
  Serial.println("\n\n✅ CALIBRAÇÃO CONCLUÍDA!");
  Serial.println("\n📋 COPIE ESTES VALORES:");
  Serial.println("========================================");
  Serial.printf("float mag_bias_x = %.4f;\n", mag_bias_x);
  Serial.printf("float mag_bias_y = %.4f;\n", mag_bias_y);
  Serial.printf("float mag_bias_z = %.4f;\n", mag_bias_z);
  Serial.printf("float mag_scale_x = %.4f;\n", mag_scale_x);
  Serial.printf("float mag_scale_y = %.4f;\n", mag_scale_y);
  Serial.printf("float mag_scale_z = %.4f;\n", mag_scale_z);
  Serial.println("========================================\n");
}

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n==========================================");
  Serial.println("   SISTEMA ICM20948 - ESP32-S3");
  Serial.println("==========================================\n");
  
  // Inicializa I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  // Inicializa sensor
  Serial.println("Procurando sensor ICM20948...");
  while (true) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok) {
      Serial.println("✅ Sensor encontrado!");
      break;
    }
    Serial.println("❌ Sensor não encontrado. Tentando novamente...");
    delay(1000);
  }
  
  // REMOVIDO: myICM.setSampleMode(...) - não necessário para funcionar
  
  // Calibração
  calibrateMagnetometer();
  
  Serial.println("\n==========================================");
  Serial.println("   SISTEMA PRONTO!");
  Serial.println("   Enviando dados pelo Serial Monitor");
  Serial.println("==========================================\n");
  delay(1000);
}

// ============================================
// LOOP PRINCIPAL
// ============================================
void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT();
    
    // Acelerômetro (g → m/s²)
    float ax = myICM.accX() * 9.80665;
    float ay = myICM.accY() * 9.80665;
    float az = myICM.accZ() * 9.80665;
    
    // Giroscópio (deg/s → rad/s)
    float gx = myICM.gyrX() * 0.0174533;
    float gy = myICM.gyrY() * 0.0174533;
    float gz = myICM.gyrZ() * 0.0174533;
    
    // Magnetômetro calibrado
    float mx = (myICM.magX() - mag_bias_x) * mag_scale_x;
    float my = (myICM.magY() - mag_bias_y) * mag_scale_y;
    float mz = (myICM.magZ() - mag_bias_z) * mag_scale_z;
    
    // Formata saída
    Serial.printf("Acel: %6.2f %6.2f %6.2f | ", ax, ay, az);
    Serial.printf("Giro: %7.3f %7.3f %7.3f | ", gx, gy, gz);
    Serial.printf("Mag: %7.1f %7.1f %7.1f\n", mx, my, mz);
  }
  
  delay(50);  // 20Hz
}
