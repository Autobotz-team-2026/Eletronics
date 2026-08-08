#include <TMCStepper.h>

// Pinos e configurações do ESP32-S3
#define RX_PIN 18 
#define TX_PIN 17 
#define SERIAL_PORT Serial1 

#define R_SENSE 0.11f       

// --- ENDEREÇOS DOS DRIVERS ---
#define ENDERECO_MOTOR_0 0b00 
#define ENDERECO_MOTOR_1 0b01 // MS1 em HIGH (3.3V/5V)

TMC2209Stepper driver0(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_0);
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_1);

void setup() {
  Serial.begin(115200);
  
  SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  driver0.pdn_disable(1);
  driver0.begin(); 
  driver1.pdn_disable(1);
  driver1.begin(); 

  // --- CONFIGURAÇÕES ESSENCIAIS ---
  driver0.toff(5); 
  driver1.toff(5); 
  
  driver0.rms_current(1000); 
  driver1.rms_current(1000); 
  
  driver0.microsteps(16);
  driver1.microsteps(16);
  
  driver0.en_spreadCycle(false); 
  driver1.en_spreadCycle(false); 
  
  driver0.pwm_autoscale(true);   
  driver1.pwm_autoscale(true);   

  // ========================================================
  // CONFIGURAÇÃO DO COOLSTEP ATUALIZADA
  // ========================================================
  
  driver0.TCOOLTHRS(0xFFFFF); 
  driver1.TCOOLTHRS(0xFFFFF); 
  
  driver0.SGTHRS(50); 
  driver1.SGTHRS(50); 


 /*====================================================================
                    FUNCIONAMENTO COMANDO SEMIN
 semin * 32 = Limite minimo -> se a leitura do StallGuard cair abaixo 
 disto o CoolStep aumenta a corrente para os 800mA totais.
======================================================================*/
  driver0.semin(5); 
  driver1.semin(5);

/*====================================================================
                    FUNCIONAMENTO COMANDO SEMAX
 (semin + semax + 1)* 32 = Limite maximo  -> Se a leitura do StallGuard 
 ultrapassar este limite o CoolStep entende que está desperdiçando energia. 
 Reduz a corrente gradativamente até chegar à metade da corrente máxima.
======================================================================*/
  driver0.semax(2); 
  driver1.semax(2); 

  driver0.sedn(0b01); // intensidade com que a corrente é reduzida/aumentada
  driver1.sedn(0b01); 

  // --- NOVAS CONFIGURAÇÕES DE ROBUSTEZ ---
  
  // 6. SEIMIN: Limite de corte. 
  // 0 = Reduz até 1/2 da corrente
  // 1 = Reduz até 1/4 da corrente
  driver0.seimin(0); 
  driver1.seimin(0); 

  // 7. SEUP: Velocidade de reação contra estol
  // 0b11 = Sobe a corrente muito rápido ao detectar peso (8 degraus por vez)
  driver0.seup(0b11); 
  driver1.seup(0b11); 

  Serial.println("Configuracao concluida. Iniciando rampa com CoolStep Aprimorado...");

  long velocidade_alvo = 20000; 
  int passo_aceleracao = 500;  
  
  for (long v = 0; v <= velocidade_alvo; v += passo_aceleracao) {
    driver0.VACTUAL(v);
    driver1.VACTUAL(v);
    delay(10); 
  }
}

 void loop() {
  
  // Lê a escala de corrente atual do motor (Ex: 7 a 14)
  uint8_t escala_corrente0 = driver0.cs_actual();
  uint8_t escala_corrente1 = driver1.cs_actual();

  // Lê qual foi o Teto Máximo (IRUN) que a biblioteca calculou para os seus 800mA (Ex: 14)
  uint8_t teto_maximo = driver0.irun(); 

  // Converte a escala bruta do chip para uma porcentagem (0% a 100%)
  // map(valor_lido, de_minimo, de_maximo, para_minimo, para_maximo)
  int porcentagem0 = map(escala_corrente0, 0, teto_maximo, 0, 100);
  int porcentagem1 = map(escala_corrente1, 0, teto_maximo, 0, 100);

  // Imprime a PORCENTAGEM no Plotter Serial
  Serial.print("Força_Motor_0(%):");
  Serial.print(porcentagem0);
  Serial.print(",");
  Serial.print("Força_Motor_1(%):");
  Serial.println(porcentagem1);

  delay(200);
}
