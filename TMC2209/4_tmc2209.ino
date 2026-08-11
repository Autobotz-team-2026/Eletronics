#include <TMCStepper.h>

// Pinos e configurações do ESP32-S3
#define RX_PIN 18 
#define TX_PIN 17 
#define SERIAL_PORT Serial1 

#define R_SENSE 0.11f        

// --- ENDEREÇOS DOS DRIVERS ---
#define ENDERECO_MOTOR_0 0b00 // MS1 = LOW (GND)  , MS2 = LOW (GND)
#define ENDERECO_MOTOR_1 0b01 // MS1 = HIGH (VCC) , MS2 = LOW (GND)
#define ENDERECO_MOTOR_2 0b10 // MS1 = LOW (GND)  , MS2 = HIGH (VCC)
#define ENDERECO_MOTOR_3 0b11 // MS1 = HIGH (VCC) , MS2 = HIGH (VCC)

// Instanciando os objetos apontando para a mesma porta serial
TMC2209Stepper driver0(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_0);
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_1);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_3);

void setup() {
  Serial.begin(115200);
  
  // Inicia a porta serial física do ESP32
  SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inicializa a comunicação lógica PRIMEIRO
  driver0.begin(); 
  driver1.begin(); 
  driver2.begin(); 
  driver3.begin();

  // AGORA desabilita a função PDN para usar os pinos apenas como endereço
  driver0.pdn_disable(1);
  driver1.pdn_disable(1);
  driver2.pdn_disable(1);
  driver3.pdn_disable(1);

  // --- CONFIGURAÇÕES ESSENCIAIS ---
  driver0.toff(5); 
  driver1.toff(5); 
  driver2.toff(5); 
  driver3.toff(5); 
  
  driver0.rms_current(1000); 
  driver1.rms_current(1000); 
  driver2.rms_current(1000); 
  driver3.rms_current(1000); 
  
  driver0.microsteps(16);
  driver1.microsteps(16);
  driver2.microsteps(16);
  driver3.microsteps(16);
  
  driver0.en_spreadCycle(false); 
  driver1.en_spreadCycle(false); 
  driver2.en_spreadCycle(false); 
  driver3.en_spreadCycle(false); 
  
  driver0.pwm_autoscale(true);   
  driver1.pwm_autoscale(true);   
  driver2.pwm_autoscale(true);   
  driver3.pwm_autoscale(true);   

  // ========================================================
  // CONFIGURAÇÃO DO COOLSTEP ATUALIZADA
  // ========================================================
  
  driver0.TCOOLTHRS(0xFFFFF); 
  driver1.TCOOLTHRS(0xFFFFF); 
  driver2.TCOOLTHRS(0xFFFFF); 
  driver3.TCOOLTHRS(0xFFFFF); 
  
  driver0.SGTHRS(50); 
  driver1.SGTHRS(50); 
  driver2.SGTHRS(50); 
  driver3.SGTHRS(50); 

  /*====================================================================
                    FUNCIONAMENTO COMANDO SEMIN
  semin * 32 = Limite minimo -> se a leitura do StallGuard cair abaixo 
  disto o CoolStep aumenta a corrente.
  ======================================================================*/
  driver0.semin(5); 
  driver1.semin(5); 
  driver2.semin(5);
  driver3.semin(5);

  /*====================================================================
                    FUNCIONAMENTO COMANDO SEMAX
  (semin + semax + 1)* 32 = Limite maximo  -> Se a leitura do StallGuard 
  ultrapassar este limite o CoolStep entende que está desperdiçando energia. 
  Reduz a corrente gradativamente até chegar à metade da corrente máxima.
  ======================================================================*/
  driver0.semax(2); 
  driver1.semax(2); 
  driver2.semax(2); 
  driver3.semax(2); 

  driver0.sedn(0b01); // intensidade com que a corrente é reduzida/aumentada
  driver1.sedn(0b01); 
  driver2.sedn(0b01); 
  driver3.sedn(0b01); 

  // --- NOVAS CONFIGURAÇÕES DE ROBUSTEZ ---
  
  // 6. SEIMIN: Limite de corte. 
  // 0 = Reduz até 1/2 da corrente
  // 1 = Reduz até 1/4 da corrente
  driver0.seimin(0); 
  driver1.seimin(0); 
  driver2.seimin(0); 
  driver3.seimin(0); 

  // 7. SEUP: Velocidade de reação contra estol
  // 0b11 = Sobe a corrente muito rápido ao detectar peso (8 degraus por vez)
  driver0.seup(0b11); 
  driver1.seup(0b11); 
  driver2.seup(0b11); 
  driver3.seup(0b11); 

  Serial.println("Configuracao concluida. Iniciando rampa com CoolStep Aprimorado em 4 Eixos...");

  long velocidade_alvo = 500; 
  int passo_aceleracao = 500;  
  
  for (long v = 0; v <= velocidade_alvo; v += passo_aceleracao) {
    driver0.VACTUAL(v);
    driver1.VACTUAL(v);
    driver2.VACTUAL(v);
    driver3.VACTUAL(v);
    delay(10); 
  }
}

void loop() {
  
  // Lê a escala de corrente atual de cada motor
  uint8_t escala_corrente0 = driver0.cs_actual();
  uint8_t escala_corrente1 = driver1.cs_actual();
  uint8_t escala_corrente2 = driver2.cs_actual();
  uint8_t escala_corrente3 = driver3.cs_actual();

  // Lê qual foi o Teto Máximo (IRUN) que a biblioteca calculou
  // (Pode ser lido apenas do driver0 já que todos têm a mesma configuração de corrente)
  uint8_t teto_maximo = driver0.irun(); 

  // Converte a escala bruta do chip para uma porcentagem (0% a 100%)
  int porcentagem0 = map(escala_corrente0, 0, teto_maximo, 0, 100);
  int porcentagem1 = map(escala_corrente1, 0, teto_maximo, 0, 100);
  int porcentagem2 = map(escala_corrente2, 0, teto_maximo, 0, 100);
  int porcentagem3 = map(escala_corrente3, 0, teto_maximo, 0, 100);

  // Imprime as porcentagens no Plotter Serial separadas por vírgula
  Serial.print("Força_Motor_0(%):");
  Serial.print(porcentagem0);
  Serial.print(",");
  Serial.print("Força_Motor_1(%):");
  Serial.print(porcentagem1);
  Serial.print(",");
  Serial.print("Força_Motor_2(%):");
  Serial.print(porcentagem2);
  Serial.print(",");
  Serial.print("Força_Motor_3(%):");
  Serial.println(porcentagem3);

  delay(200);
}