#include <Wire.h>

// Pinos de comunicação I2C entre o ESP32 e a entrada da PCA
#define PIN_SDA 8
#define PIN_SCL 7

// Endereços I2C padrões
#define PCA_ADDR 0x70  // Endereço da placa roxa
#define HMC_ADDR 0x1E  // Endereço padrão do magnetômetro HMC5883L

// Função para selecionar o canal na placa roxa
void selecionarCanalPCA(uint8_t canal) {
  if (canal > 7) return;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial); 

  // Inicializa o barramento I2C principal nos pinos 8 e 7
  Wire.begin(PIN_SDA, PIN_SCL);
  Serial.println("\n--- INICIANDO TESTE SEGURO: TCA9548A (CANAL 1) + HMC5883L ---");

  // 1. Abre o Canal 1 (Pinos SD1 e SC1 que você conectou)
  Serial.println("Abrindo o Canal 1 na placa roxa...");
  selecionarCanalPCA(1);
  delay(100);

  // 2. Tenta comunicação com o Magnetômetro
  Serial.println("Tentando comunicar com o magnetometro HMC5883L...");
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x00); // Registrador de Configuração A
  Wire.write(0x70); // Taxa de amostragem padrão (15Hz)
  
  if (Wire.endTransmission() != 0) {
    Serial.println("\n[AVISO] Magnetometro nao encontrado no Canal 1.");
    Serial.println("Se os fios de energia estao certos, isso significa apenas que esses furos");
    Serial.println("superiores levam a outro conector (como o J10 ou J12). Nao ha risco de queimar!");
    while (1); 
  }

  // 3. Coloca o HMC5883L em modo de medição contínua
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x02); // Registrador de Modo
  Wire.write(0x00); // Modo contínuo
  Wire.endTransmission();

  Serial.println("[SUCESSO] Sistema conectado e configurado com seguranca!");
}

void loop() {
  // Garante que o Canal 1 permanece ativo
  selecionarCanalPCA(1);

  // Solicita a leitura dos eixos
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x03); 
  Wire.endTransmission(false);
  
  Wire.requestFrom(HMC_ADDR, 6);

  if (Wire.available() == 6) {
    int16_t x = (Wire.read() << 8) | Wire.read();
    int16_t z = (Wire.read() << 8) | Wire.read();
    int16_t y = (Wire.read() << 8) | Wire.read();

    Serial.print("Magnetometro -> X: "); Serial.print(x);
    Serial.print(" | Y: "); Serial.print(y);
    Serial.print(" | Z: "); Serial.println(z);
  } else {
    Serial.println("[ALERTA] Aguardando resposta do sensor...");
  }

  delay(300); 
}
