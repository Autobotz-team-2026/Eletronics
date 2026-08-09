#include <Wire.h>

// Pinos e Endereços I2C
#define SDA_PIN 8
#define SCL_PIN 46
#define MPU_ADDR 0x68
#define MAG_ADDR 0x0C

// Registradores
#define PWR_MGMT_1   0x6B
#define GYRO_CONFIG  0x1B
#define INT_PIN_CFG  0x37
#define GYRO_ZOUT_H  0x47
#define MAG_CNTL1    0x0A
#define MAG_HXL      0x03

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 
  delay(100);

  Serial.println("\n==============================================");
  Serial.println("  FERRAMENTA DE CALIBRACAO DO MPU9250 / AK8963  ");
  Serial.println("==============================================\n");

  // 1. Acorda MPU6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);

  // 2. Força a escala MÁXIMA do Giroscópio (+/- 2000 graus/s) para bater com seu código principal
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission();
  delay(50);

  // 3. Ativa Bypass Mode
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(50);

  // 4. Configura o Magnetômetro
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(MAG_CNTL1);
  Wire.write(0x16);
  Wire.endTransmission();
  delay(50);

  // ==========================================================
  // ETAPA 1: CALIBRAÇÃO DO GIROSCÓPIO
  // ==========================================================
  Serial.println(">>> ETAPA 1: GIROSCOPIO <<<");
  Serial.println("MANTENHA O SENSOR TOTALMENTE IMOVEL NA MESA!");
  Serial.println("Iniciando em 3 segundos...");
  delay(3000);
  
  long somaGyro = 0;
  int amostras = 500; // Pegamos 500 amostras para ter alta precisão
  
  for (int i = 0; i < amostras; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_ZOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2);
    int16_t gz = Wire.read() << 8 | Wire.read();
    somaGyro += gz;
    delay(5);
  }
  
  float gyroZ_offset = (float)somaGyro / (float)amostras;
  Serial.println("Giroscopio calibrado com sucesso!\n");
  delay(2000);


  // ==========================================================
  // ETAPA 2: CALIBRAÇÃO DO MAGNETÔMETRO
  // ==========================================================
  Serial.println(">>> ETAPA 2: MAGNETOMETRO <<<");
  Serial.println("Quando iniciar, gire o sensor em formato de '8' no ar.");
  Serial.println("Faca o movimento cobrindo todos os eixos X, Y e Z.");
  Serial.println("Iniciando em 5 segundos...");
  delay(5000);
  
  Serial.println("\n*** GRAVANDO! GIRE O SENSOR AGORA! (15 segundos) ***");
  
  int32_t mag_max[3] = {-32767, -32767, -32767};
  int32_t mag_min[3] = {32767, 32767, 32767};
  unsigned long tempoInicio = millis();
  
  while (millis() - tempoInicio < 15000) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(MAG_HXL);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 7);
    
    if (Wire.available() >= 7) {
      int16_t x = Wire.read() | (Wire.read() << 8);
      int16_t y = Wire.read() | (Wire.read() << 8);
      int16_t z = Wire.read() | (Wire.read() << 8);
      Wire.read(); // ST2
      
      if (x > mag_max[0]) mag_max[0] = x;
      if (x < mag_min[0]) mag_min[0] = x;
      if (y > mag_max[1]) mag_max[1] = y;
      if (y < mag_min[1]) mag_min[1] = y;
      if (z > mag_max[2]) mag_max[2] = z;
      if (z < mag_min[2]) mag_min[2] = z;
    }
    
    // Mostra uma contagem visual na tela para você saber que está rodando
    if (millis() % 1000 < 10) {
      Serial.print(".");
    }
    delay(10);
  }

  // Cálculos de Hard-Iron (Bias) e Soft-Iron (Scale)
  float mag_bias[3];
  float mag_scale[3];
  
  for (int i = 0; i < 3; i++) {
    mag_bias[i] = (mag_max[i] + mag_min[i]) / 2.0;
  }
  
  float deltaX = (mag_max[0] - mag_min[0]) / 2.0;
  float deltaY = (mag_max[1] - mag_min[1]) / 2.0;
  float deltaZ = (mag_max[2] - mag_min[2]) / 2.0;
  float avg_delta = (deltaX + deltaY + deltaZ) / 3.0;
  
  mag_scale[0] = avg_delta / deltaX;
  mag_scale[1] = avg_delta / deltaY;
  mag_scale[2] = avg_delta / deltaZ;


  // ==========================================================
  // ETAPA 3: IMPRESSÃO DOS RESULTADOS PRONTOS PARA CÓPIA
  // ==========================================================
  Serial.println("\n\n==============================================");
  Serial.println("  CALIBRACAO CONCLUIDA! COPIE O BLOCO ABAIXO:   ");
  Serial.println("==============================================\n");
  
  Serial.println("// =======================================================");
  Serial.println("// AJUSTES MANUAIS DE CALIBRACAO (Altere aqui!)");
  Serial.println("// =======================================================");
  
  Serial.print("float manual_gyroZ_offset = ");
  Serial.print(gyroZ_offset, 4);
  Serial.println("; ");
  
  Serial.println("");
  Serial.print("float manual_mag_bias_x = "); Serial.print(mag_bias[0], 4); Serial.println(";");
  Serial.print("float manual_mag_bias_y = "); Serial.print(mag_bias[1], 4); Serial.println(";");
  Serial.print("float manual_mag_bias_z = "); Serial.print(mag_bias[2], 4); Serial.println(";");
  
  Serial.println("");
  Serial.print("float manual_mag_scale_x = "); Serial.print(mag_scale[0], 4); Serial.println(";");
  Serial.print("float manual_mag_scale_y = "); Serial.print(mag_scale[1], 4); Serial.println(";");
  Serial.print("float manual_mag_scale_z = "); Serial.print(mag_scale[2], 4); Serial.println(";");
  
  Serial.println("\n// =======================================================");
}

void loop() {
  // O código para aqui.
}
