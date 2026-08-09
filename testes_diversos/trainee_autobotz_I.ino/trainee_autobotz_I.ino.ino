#define ECHO 6  // variaveis uteis
#define PIR 7
int verde_rua = 13;
int amarelo_rua = 12;
int vermelho_rua = 11;
int vermelho_ave = 10;
int amarelo_ave = 9;
int verde_ave = 8;
long cm = 0;

long lerDistancia(int pinoPulso, int pinoEcho) {  //função de ler a distancia
  pinMode(pinoPulso, OUTPUT);

  digitalWrite(pinoPulso, LOW);

  delayMicroseconds(2);

  digitalWrite(pinoPulso, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinoPulso, LOW);

  pinMode(pinoEcho, INPUT);  //nao sei pq isso aq nao pode ir na linha 13
  return pulseIn(pinoEcho, HIGH);
}

void setup() {
  //Serial.begin(9600);
  pinMode(verde_rua,    OUTPUT);
  pinMode(vermelho_rua, OUTPUT);
  pinMode(amarelo_rua,  OUTPUT);
  pinMode(verde_ave,    OUTPUT);
  pinMode(amarelo_ave,  OUTPUT);
  pinMode(vermelho_ave, OUTPUT);
  pinMode(PIR,          INPUT);
}

void loop() {
  cm = 0.0172 * lerDistancia(ECHO, ECHO);
  // Serial.println(cm); //desnecessário, apenas para controle
  // delay(1000);

  if (!((cm <= 7) && (digitalRead(PIR) == LOW))) {
    digitalWrite(verde_rua, LOW);
    digitalWrite(amarelo_rua, LOW);
    digitalWrite(vermelho_rua, HIGH);

    digitalWrite(verde_ave, HIGH);
    digitalWrite(amarelo_ave, LOW);
    digitalWrite(vermelho_ave, LOW);

  } else {

    digitalWrite(verde_ave, LOW);
    digitalWrite(amarelo_ave, HIGH);
    digitalWrite(vermelho_ave, LOW);
    delay(3000);

    digitalWrite(verde_ave, LOW);
    digitalWrite(amarelo_ave, LOW);
    digitalWrite(vermelho_ave, HIGH);

    digitalWrite(verde_rua, HIGH);
    digitalWrite(amarelo_rua, LOW);
    digitalWrite(vermelho_rua, LOW);
    delay(10000);

    digitalWrite(verde_rua, LOW);
    digitalWrite(amarelo_rua, HIGH);
    digitalWrite(vermelho_rua, LOW);
    delay(3000);

    digitalWrite(verde_rua, LOW);
    digitalWrite(amarelo_rua, LOW);
    digitalWrite(vermelho_rua, HIGH);

    digitalWrite(verde_ave, HIGH);
    digitalWrite(amarelo_ave, LOW);
    digitalWrite(vermelho_ave, LOW);
    delay(1000);
  }
}
