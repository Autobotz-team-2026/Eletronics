#include <Wire.h>
#include "ICM_20948.h"

#define SDA_PIN 8 
#define SCL_PIN 46
#define AD0_VAL 0 

ICM_20948_I2C myICM;

// Variáveis para guardar o erro magnético do ambiente
float mag_bias_x = 0;
float mag_bias_y = 0;
float mag_bias_z = 0;

int static_variable = 500;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); 

  bool connected = false;
  while (!connected) {
    myICM.begin(Wire, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Procurando o sensor no I2C...");
      delay(500);
    } else {
      connected = true;
    }
  }
  Serial.println("Sensor conectado!");

  // --- ROTINA DE CALIBRAÇÃO MAGNÉTICA (HARD IRON) ---
  Serial.println("\n==========================================");
  Serial.println("INICIANDO CALIBRAÇÃO MAGNÉTICA EM 3 SEG...");
  Serial.println("PREPARE-SE PARA GIRAR O SENSOR EM TODAS AS DIREÇÕES!");
  delay(3000);
  
  float mag_min[3] = {32767, 32767, 32767};
  float mag_max[3] = {-32768, -32768, -32768};
  
  unsigned long tempo_calibracao = millis();
  
  // Fica 10 segundos coletando os dados extremos
  while(millis() - tempo_calibracao < 10000) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      
      // Coleta
      float mx = myICM.magX();
      float my = myICM.magY();
      float mz = myICM.magZ();
      
      // Atualiza Mínimos
      if (mx < mag_min[0]) mag_min[0] = mx;
      if (my < mag_min[1]) mag_min[1] = my;
      if (mz < mag_min[2]) mag_min[2] = mz;
      
      // Atualiza Máximos
      if (mx > mag_max[0]) mag_max[0] = mx;
      if (my > mag_max[1]) mag_max[1] = my;
      if (mz > mag_max[2]) mag_max[2] = mz;
    }
  }

  // O "Bias" (erro) é o ponto central entre o máximo e o mínimo encontrado
  mag_bias_x = (mag_max[0] + mag_min[0]) / 2.0;
  mag_bias_y = (mag_max[1] + mag_min[1]) / 2.0;
  mag_bias_z = (mag_max[2] + mag_min[2]) / 2.0;

  Serial.println("CALIBRAÇÃO CONCLUÍDA!");
  Serial.println("Erros detectados (Offsets):");
  Serial.print("X: "); Serial.print(mag_bias_x);
  Serial.print(" | Y: "); Serial.print(mag_bias_y);
  Serial.print(" | Z: "); Serial.println(mag_bias_z);
  Serial.println("==========================================\n");
  delay(2000);
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT(); 

    float ax = myICM.accX();
    float ay = myICM.accY();
    float az = myICM.accZ();

    // Extrai o campo magnético e SUBTRAI o erro (Bias) que descobrimos na calibração
    float mx = myICM.magX() - mag_bias_x;
    float my = myICM.magY() - mag_bias_y;
    float mz = myICM.magZ() - mag_bias_z;

    // Atenção: A documentação da InvenSense relata que o eixo do compasso interno 
    // é rotacionado em relação ao acelerômetro. Vamos mapear os eixos corretamente:
    float compass_x = my;
    float compass_y = mx;
    float compass_z = -mz;

    float roll  = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // Aplica a matriz de Compensação de Inclinação usando os eixos corrigidos
    float X_comp = compass_x * cos(pitch) + compass_z * sin(pitch);
    float Y_comp = compass_x * sin(roll) * sin(pitch) + compass_y * cos(roll) - compass_z * sin(roll) * cos(pitch);

    float yaw = atan2(Y_comp, X_comp) * 180.0 / PI;

    if (yaw < 0) {
      yaw += 360.0;
    }

    // Serial.print("Yaw Calibrado: ");
    // Serial.print(yaw);
    // Serial.println(" °");
  
    Serial.print("Variable_1:");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print("Variable_2:");
    Serial.println(static_variable);
    
    delay(50);
  }
}