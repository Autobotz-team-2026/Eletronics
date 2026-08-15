// Definição dos pinos de controle do Motor A no L298N
const int pinIN1 = 8;
const int pinIN2 = 7;
const int pinENA = 9; // Pino PWM para controle de velocidade

// Configuração do tempo de aceleração
// Quanto maior o delay, mais suave (e demorada) é a aceleração
// o tempo de aceleracao será (tempoRampa x 255)
const int tempoRampa = 5; 

void setup() {
  // Configura os pinos como saída
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENA, OUTPUT);

  // Garante que o motor comece desligado
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  analogWrite(pinENA, 0);
}

void loop() {
  // 1. Define o sentido de rotação (Horário)
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  
  // 2. Aceleração (Soft Start)
  // Vai incrementando o PWM de 0 (parado) até 255 (velocidade máxima)
  for (int velocidade = 0; velocidade <= 255; velocidade++) {
    analogWrite(pinENA, velocidade);
    delay(tempoRampa); 
  }

  // Mantém o motor em velocidade máxima por 2 segundos
  delay(10000);

  // 3. Desaceleração (Soft Stop)
  // Vai decrementando o PWM de 255 até 0
  for (int velocidade = 255; velocidade >= 0; velocidade--) {
    analogWrite(pinENA, velocidade);
    delay(tempoRampa);
  }

  // Pausa com o motor parado antes de inverter o sentido
  delay(2000);

  // 4. Inverte o sentido de rotação (Anti-horário)
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, HIGH);

  // 5. Repete a aceleração para o outro lado
  for (int velocidade = 0; velocidade <= 255; velocidade++) {
    analogWrite(pinENA, velocidade);
    delay(tempoRampa); 
  }

  delay(10000);

  // 6. Repete a desaceleração
  for (int velocidade = 255; velocidade >= 0; velocidade--) {
    analogWrite(pinENA, velocidade);
    delay(tempoRampa);
  }

  // Pausa maior antes de recomeçar o ciclo
  delay(2000);
}
