#include <TMCStepper.h>

// Pinos e configurações do ESP32-S3
#define RX_PIN 18 
#define TX_PIN 17 
#define SERIAL_PORT Serial1 

#define R_SENSE 0.11f       

// --- ENDEREÇOS DOS DRIVERS ---
#define ENDERECO_MOTOR_0 0b00 // MS1 e MS2 em LOW
#define ENDERECO_MOTOR_1 0b01 // MS1 em HIGH (3.3V/5V) e MS2 em LOW

/*==============================================================
  O foco deste código é demonstrar que é possível dar comandos 
  de STEP e DIREÇÃO via uart para cada driver individualmente, 
  sem utilizar os pinos STEP e DIR do motor e, mais importante, 
  sem ser necessário uma biblioteca como a AccelStepper.
*/=============================================================


// Instanciando os dois drivers
TMC2209Stepper driver0(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_0);
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_1);

// Variáveis globais para controlar a velocidade e a rampa
long velocidade_atual = 0;
long velocidade_alvo = 10000; // Velocidade máxima
int passo_aceleracao = 1000;  // Quão rápido ele acelera/freia

// Função inteligente de rampa para DOIS motores espelhados
void mudarVelocidade(long novo_alvo) {
  if (velocidade_atual < novo_alvo) {
    // Precisa acelerar (ou frear do negativo para o zero)
    for (long v = velocidade_atual; v <= novo_alvo; v += passo_aceleracao) {
      driver0.VACTUAL(v);
      driver1.VACTUAL(-v); // O Motor 1 sempre recebe o valor matematicamente invertido!
      delay(5); 
    }
  } else {
    // Precisa desacelerar (ou acelerar para o sentido negativo)
    for (long v = velocidade_atual; v >= novo_alvo; v -= passo_aceleracao) {
      driver0.VACTUAL(v);
      driver1.VACTUAL(-v); // O Motor 1 sempre recebe o valor matematicamente invertido!
      delay(5); 
    }
  }
  
  // Atualiza a variável global e garante que cravou exatamente no alvo final
  velocidade_atual = novo_alvo;
  driver0.VACTUAL(velocidade_atual); 
  driver1.VACTUAL(-velocidade_atual); 
}

void setup() {
  Serial.begin(115200);
  
  SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  driver0.pdn_disable(1);
  driver0.begin(); 
  driver1.pdn_disable(1);
  driver1.begin(); 

  // --- CONFIGURAÇÕES ESSENCIAIS PARA AMBOS OS MOTORES ---
  driver0.toff(5); 
  driver1.toff(5); 
  
  driver0.rms_current(800); 
  driver1.rms_current(800); 
  
  driver0.microsteps(16);
  driver1.microsteps(16);
  
  driver0.en_spreadCycle(true); 
  driver1.en_spreadCycle(true); 
  
  driver0.pwm_autoscale(true);   
  driver1.pwm_autoscale(true);   

  Serial.println("Configuracao concluida. Iniciando motores em sentidos opostos...");

  // Dá a partida inicial suave
  mudarVelocidade(velocidade_alvo);
}

void loop() {
  // Mantém girando por 5 segundos
  delay(5000); 

  Serial.println("Invertendo direcao de AMBOS os motores...");
  // Manda a função ir para a velocidade negativa 
  // (Motor 0 vai para negativo, Motor 1 vai para positivo)
  mudarVelocidade(-velocidade_alvo); 

  // Mantém girando na nova direção por 5 segundos
  delay(5000); 

  Serial.println("Invertendo direcao de AMBOS os motores novamente...");
  // Manda a função voltar para a configuração inicial
  mudarVelocidade(velocidade_alvo); 
}