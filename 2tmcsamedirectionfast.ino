#include <TMCStepper.h>

// Pinos e configurações do ESP32-S3
#define RX_PIN 18 
#define TX_PIN 17 
#define SERIAL_PORT Serial1 

#define R_SENSE 0.11f       

// --- ENDEREÇOS DOS DRIVERS ---
#define ENDERECO_MOTOR_0 0b00 // MS1 e MS2 em LOW (ou desconectados)
#define ENDERECO_MOTOR_1 0b01 // MS1 em HIGH (3.3V/5V) e MS2 em LOW

// Instanciando os dois drivers na mesma porta serial
TMC2209Stepper driver0(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_0);
TMC2209Stepper driver1(&SERIAL_PORT, R_SENSE, ENDERECO_MOTOR_1);

void setup() {
  Serial.begin(115200);
  
  // Inicia a porta serial de hardware para os TMC2209
  SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inicializa o Driver 0
  driver0.pdn_disable(1);
  driver0.begin(); 
  
  // Inicializa o Driver 1
  driver1.pdn_disable(1);
  driver1.begin(); 

  // --- CONFIGURAÇÕES ESSENCIAIS PARA AMBOS OS MOTORES ---
  
  // Habilita os estágios de potência
  driver0.toff(5); 
  driver1.toff(5); 
  
  // Define a corrente (800mA)
  driver0.rms_current(800); 
  driver1.rms_current(800); 
  
  // Define a resolução (16 micropassos)
  driver0.microsteps(16);
  driver1.microsteps(16);
  
  // Habilita SpreadCycle para torque em alta velocidade
  driver0.en_spreadCycle(true); 
  driver1.en_spreadCycle(true); 
  
  // Autoscale do PWM
  driver0.pwm_autoscale(true);   
  driver1.pwm_autoscale(true);   

  Serial.println("Configuracao concluida. Iniciando rampa de aceleracao sincronizada...");

  // --- RAMPA DE ACELERAÇÃO (PARA OS DOIS MOTORES) ---
  
  long velocidade_alvo = 60000; 
  int passo_aceleracao = 1000;  
  
  for (long v = 0; v <= velocidade_alvo; v += passo_aceleracao) {
    // Envia o comando de velocidade para ambos os motores quase simultaneamente
    driver0.VACTUAL(v);
    driver1.VACTUAL(v);
    
    // Tempo para os motores fisicamente alcançarem a velocidade magnética
    delay(15); 
  }
  
  Serial.println("Velocidade maxima atingida em ambos os motores!");
}

void loop() {
  // Os motores continuarão girando na velocidade máxima definida pela rampa.
}