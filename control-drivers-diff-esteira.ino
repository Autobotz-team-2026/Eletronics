// =========================================================================
// DEFINIÇÃO DOS PINOS
// =========================================================================

// Motor 1 - Driver L298N
const int ENA = 6;  // Controle de velocidade (PWM)
const int IN1 = 5;  // Sentido de rotação A
const int IN2 = 4;  // Sentido de rotação B

// Motor 2 - Driver BTS7960
const int RPWM = 17;   // PWM para frente (Right)
const int LPWM = 19;   // PWM para trás (Left)

const int L_EN= 20; // Habilita as pontas (R_EN e L_EN interligados)
const int R_EN = 18;

void setup() {
  // Inicializa comunicação serial para testes
  Serial.begin(115200);

  // Configuração dos pinos do L298N
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Configuração dos pinos do BTS7960
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);

  // Ativa o driver BTS7960 liberando os pinos de Enable
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  Serial.println("Drivers prontos e inicializados!");
}

void loop() {
  // --- EXEMPLO 1: Ambos os motores para frente (Velocidade Média/Alta) ---
  Serial.println("Motores para FRENTE...");
  controlarL298N(200);     // Valores de -255 a 255
  controlarBTS7960(200);   // Valores de -255 a 255
  delay(3000);             // Mantém por 3 segundos

  // --- EXEMPLO 2: Parar os motores ---
  Serial.println("Motores PARADOS.");
  controlarL298N(0);
  controlarBTS7960(0);
  delay(1500);             // Mantém parado por 1.5 segundos

  // --- EXEMPLO 3: Ambos os motores para trás (Velocidade Baixa) ---
  Serial.println("Motores em REVERSO...");
  controlarL298N(-120);    // Valor negativo aciona o sentido inverso
  controlarBTS7960(-120);
  delay(3000);             // Mantém por 3 segundos

  // --- EXEMPLO 4: Parar os motores antes de reiniciar o ciclo ---
  controlarL298N(0);
  controlarBTS7960(0);
  delay(2000);
}

// =========================================================================
// FUNÇÕES DE CONTROLE
// =========================================================================

/**
 * Controla o motor conectado ao L298N.
 * @param velocidade Aceita valores de -255 (reverso máximo) a 255 (frente máxima). 0 para parar.
 */
void controlarL298N(int velocidade) {
  // Limita a velocidade entre os extremos permitidos
  velocidade = constrain(velocidade, -255, 255);

  if (velocidade > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, velocidade);
  } 
  else if (velocidade < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(velocidade)); // abs converte o valor negativo em positivo
  } 
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

/**
 * Controla o motor conectado ao BTS7960.
 * @param velocidade Aceita valores de -255 (reverso máximo) a 255 (frente máxima). 0 para parar.
 */
void controlarBTS7960(int velocidade) {
  // Limita a velocidade entre os extremos permitidos
  velocidade = constrain(velocidade, -255, 255);

  if (velocidade > 0) {
    analogWrite(LPWM, 0);               // Garante que o outro lado está zerado
    analogWrite(RPWM, velocidade);      // Aplica o sinal no pino de avanço
  } 
  else if (velocidade < 0) {
    analogWrite(RPWM, 0);               // Garante que o outro lado está zerado
    analogWrite(LPWM, abs(velocidade)); // Aplica o sinal no pino de recuo
  } 
  else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}