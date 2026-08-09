#include <TMCStepper.h>

// Pinos e configurações do ESP32-S3
#define RX_PIN 18 
#define TX_PIN 17 
#define SERIAL_PORT Serial1 

#define R_SENSE 0.11f       

// --- ENDEREÇOS DOS DRIVERS ---
#define ENDERECO_MOTOR_0 0b00 
#define ENDERECO_MOTOR_1 0b01 // Lembre-se: MS1 no 3.3V/5V!

TMC2209Stepper driver0(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_0);
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_1);

// Variáveis da Rampa (Reduzidas para o StealthChop funcionar sem travar)
long velocidade_atual = 0;
long velocidade_alvo = 30000; // Velocidade máxima reduzida
int passo_aceleracao = 1000;   // Aceleração mais suave
int direcao_atual = 1;        // 1 para frente, -1 para trás

// Timers não-bloqueantes para o loop
unsigned long timer_leitura = 0;
unsigned long timer_inversao = 0;

// Função inteligente de rampa para DOIS motores espelhados
void mudarVelocidade(long novo_alvo) {
  if (velocidade_atual < novo_alvo) {
    for (long v = velocidade_atual; v <= novo_alvo; v += passo_aceleracao) {
      driver0.VACTUAL(v);
      driver1.VACTUAL(-v); 
      delay(2); // Reduzido para agilizar a leitura
    }
  } else {
    for (long v = velocidade_atual; v >= novo_alvo; v -= passo_aceleracao) {
      driver0.VACTUAL(v);
      driver1.VACTUAL(-v); 
      delay(2); 
    }
  }
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

  // Configurações Base
  driver0.toff(5); 
  driver1.toff(5); 
  driver0.rms_current(1000); 
  driver1.rms_current(1000); 
  driver0.microsteps(16);
  driver1.microsteps(16);
  
  // --- CONFIGURAÇÃO PARA O STALLGUARD ---
  
  driver0.en_spreadCycle(true); 
  driver1.en_spreadCycle(true); 
  
  driver0.pwm_autoscale(true);   
  driver1.pwm_autoscale(true);   

  // 2. Definir o limiar de velocidade (TCOOLTHRS) para ambos
  driver0.TCOOLTHRS(0xFFFFF); 
  driver1.TCOOLTHRS(0xFFFFF); 
  
  // 3. Sensibilidade do Stall (0 a 255)
  driver0.SGTHRS(50); 
  driver1.SGTHRS(50); 

  Serial.println("Configuracao concluida. Iniciando rampa suave...");
  mudarVelocidade(velocidade_alvo);
}

void loop() {
  unsigned long tempo_atual = millis();

  // ========================================================
  // TAREFA 1: Ler e imprimir no formato do SERIAL PLOTTER
  // ========================================================
  if (tempo_atual - timer_leitura >= 100) {
    uint16_t stall0 = driver0.SG_RESULT();
    uint16_t stall1 = driver1.SG_RESULT();

    // FORMATO PARA O PLOTTER: "Nome1:Valor1, Nome2:Valor2"
    
    Serial.print("Motor_0:"); // Nome na legenda
    Serial.print(stall0);     // Valor da linha 1
    
    Serial.print(",");        // Virgula para separar as variaveis no grafico
    
    Serial.print("Motor_1:"); // Nome na legenda
    Serial.println(stall1);   // Valor da linha 2 (println pula a linha pro proximo ciclo)

    timer_leitura = tempo_atual; 
  }

  // ========================================================
  // TAREFA 2: Inverter a direção a cada 5 segundos (5000ms)
  // ========================================================
  if (tempo_atual - timer_inversao >= 5000) {
    direcao_atual = -direcao_atual; 
    mudarVelocidade(velocidade_alvo * direcao_atual);
    timer_inversao = millis(); 
  }
}