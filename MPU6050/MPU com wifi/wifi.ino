#include <WiFi.h>
#include <Wire.h>
#include <WiFiManager.h>
#include "PaginaWeb.h"
#include "OTA.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Kalman.h>


TwoWire I2C_Dois = TwoWire(1);
MPU_Data m1, m2, m3;
SemaphoreHandle_t xMutex;

// --- TAREFA DE PROCESSAMENTO (CORE 0) ---
void TaskKalman(void *pvParameters) {
  Wire.begin(21, 22);
  I2C_Dois.begin(33, 32); 

  // Tenta iniciar os sensores e guarda se deu certo (true) ou erro (false)
  bool mpu1_ok = m1.mpu.begin(0x68, &Wire);
  bool mpu2_ok = m2.mpu.begin(0x69, &Wire);
  bool mpu3_ok = m3.mpu.begin(0x68, &I2C_Dois);

  sensors_event_t a, g, t;
  
  m1.timer = m2.timer = m3.timer = micros();
  
  for(;;) {
    if (xSemaphoreTake(xMutex, (TickType_t)5)) {
        
        // Só tenta ler o MPU 1 se ele estiver conectado
        if (mpu1_ok) {
            m1.mpu.getEvent(&a, &g, &t);
            double dt = (double)(micros() - m1.timer) / 1000000;
            m1.timer = micros();
            float roll = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
            m1.anguloFiltrado = m1.kalman.getAngle(roll, g.gyro.x * RAD_TO_DEG, dt);
        }

        // Só tenta ler o MPU 2 se ele estiver conectado
        if (mpu2_ok) {
            m2.mpu.getEvent(&a, &g, &t);
            double dt = (double)(micros() - m2.timer) / 1000000;
            m2.timer = micros();
            float roll = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
            m2.anguloFiltrado = m2.kalman.getAngle(roll, g.gyro.x * RAD_TO_DEG, dt);
        }

        // Só tenta ler o MPU 3 se ele estiver conectado
        if (mpu3_ok) {
            m3.mpu.getEvent(&a, &g, &t);
            double dt = (double)(micros() - m3.timer) / 1000000;
            m3.timer = micros();
            float roll = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
            m3.anguloFiltrado = m3.kalman.getAngle(roll, g.gyro.x * RAD_TO_DEG, dt);
        }
        
        xSemaphoreGive(xMutex);
    }
    
    // O respiro vital para o Watchdog não resetar a placa!
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}


void setup() {
  xMutex = xSemaphoreCreateMutex();

  setupOTA();

  setupWebServer();

  // 3. Criar a tarefa no Core 0
  xTaskCreatePinnedToCore(TaskKalman, "KalmanTask", 8192, NULL, 2, NULL, 0);

  Serial.println("\n\n========================================");
  Serial.println("      🚀 SISTEMA AUTOBOTZ INICIADO      ");
  Serial.println("========================================");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[ STATUS DE CONEXÃO ]");
    Serial.print("  🌐 Rede:      "); Serial.println(WiFi.SSID());
    Serial.print("  📍 IP Local: http://"); Serial.println(WiFi.localIP());
    Serial.println("----------------------------------------");
    Serial.println("✅ Servidor Web e Dashboard prontos!");
    Serial.println("========================================\n");
  }
}

void loop() {

  ArduinoOTA.handle();
  //wm.resetSettings();

  if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Conexão Wi-Fi perdida! Abrindo portal de reconexão...");
      stopWebServer();
      wm.startConfigPortal("RECONECTAR_AUTOBOTZ", "12345678"); 
      delay(1000);
      ESP.restart();
  }

  loopWebServer(); 
}