#include <TMCStepper.h>

// --- Configurações de Pinos ---
#define EN_PIN           15 
#define DIR_PIN          14 
#define STEP_PIN         12 
#define RX_PIN           16 
#define TX_PIN           17 

#define SERIAL_PORT      Serial2 
#define DRIVER_ADDRESS   0b00 
#define R_SENSE          0.11f 

// --- Configurações de Velocidade e Stall ---
#define STALL_VALUE      100 // sensibilidade do Detector de Stall
uint32_t frequency_hz = 2000;  

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);


hw_timer_t * timer = NULL;
volatile bool motor_enabled = true;

void IRAM_ATTR onTimer() {
  if (motor_enabled) {
    digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
  }
}

void setup() {
  Serial.begin(115200);
  SERIAL_PORT.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // inicialização uart

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);

  driver.begin();
  driver.toff(4);
  driver.microsteps(2); // seleção de microsteps desejada
  driver.I_scale_analog(0); // altera a referência da corrente para setada no código
  driver.TCOOLTHRS(0xFFFFF);
  driver.SGTHRS(STALL_VALUE); // sensibilidade do stall
  driver.en_spreadCycle(false); // modo ruidoso (spreadcycle) ou silencioso (stealhchop)
  driver.rms_current(600);// corrente desejada
  timer = timerBegin(1000000); // Timer base de 1MHz
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / frequency_hz, true, 0); 

  Serial.println("Driver Iniciado (Timer v3.x)");
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t ms = millis();

  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '0') {
      motor_enabled = false;
      digitalWrite(EN_PIN, HIGH);
    } 
    else if (c == '1') {
      motor_enabled = true;
      digitalWrite(EN_PIN, LOW);
    } 
    else if (c == '+') {
      frequency_hz += 200;
      timerAlarm(timer, 1000000 / frequency_hz, true, 0);
      Serial.print("Freq: "); Serial.println(frequency_hz);
    } 
    else if (c == '-') {
      if (frequency_hz > 400) frequency_hz -= 200;
      timerAlarm(timer, 1000000 / frequency_hz, true, 0);
      Serial.print("Freq: "); Serial.println(frequency_hz);
    }
  }

  if ((ms - last_time) > 100) {
    last_time = ms;
    Serial.print("SG_Val:");
    Serial.println(driver.SG_RESULT());
  }
}
