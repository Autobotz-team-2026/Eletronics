int LED_PIN = 2;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.print("ESP32-S3 VIVO E ENVIANDO DADOS!, ");
  Serial.println(LED_PIN);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}
