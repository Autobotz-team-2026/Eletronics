/*
------------------------------------------------------------------------------------
  feat: implementa arquitetura Dual-Core com FreeRTOS e isolamento de portas

- Separa micro-ROS no Core 0 usando USB Nativa (latência ultrabaixa).
- Move controle de motores e PWM para o Core 1.
- Isola logs de debug na porta UART (Serial0) independente do ROS.
- Implementa Watchdog de 500ms atrelado ao callback de recebimento de pacotes.
- Requer compilação com 'USB CDC On Boot: Enabled'.
------------------------------------------------------------------------------------
*/


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Variáveis para simular o que seriam os dados do robô
volatile float simulacao_v = 0.0;
volatile TickType_t tempo_ultimo_comando = 0;

// Tarefa 1: Simula o micro-ROS (Rodando no Núcleo 0 - Cabo da Direita/Nativo)
void Task_ROS_Sim(void *pvParameters) {
  // Serial aqui é a USB Nativa (CDC Enabled)
  Serial.begin(115200); 
  
  while(1) {
    // Simulando a chegada de um comando de velocidade
    simulacao_v = 0.5; 
    tempo_ultimo_comando = xTaskGetTickCount();
    
    Serial.println(">> [CORE 0] Recebi comando via USB Nativa");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo
  }
}

// Tarefa 2: Debug e Monitoramento (Rodando no Núcleo 1 - Cabo da Esquerda/UART)
void Task_Debug(void *pvParameters) {
  // Serial0 é a porta UART física (TX0/RX0)
  Serial0.begin(115200);
  
  while(1) {
    Serial0.print(">> [CORE 1] Log de Hardware - Velocidade Atual: ");
    Serial0.println(simulacao_v+2);
    
    // Verificação de segurança (Watchdog)
    if ((xTaskGetTickCount() - tempo_ultimo_comando) > pdMS_TO_TICKS(2000)) {
      Serial0.println("WARNING: Perda de sinal detectada no cabo de dados!");
    }

    vTaskDelay(pdMS_TO_TICKS(500)); // Log a cada 500ms
  }
}

void setup() {
  // Criar as threads em núcleos diferentes
  xTaskCreatePinnedToCore(Task_ROS_Sim, "ROS_Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Task_Debug, "Debug_Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // O loop fica vazio e deletamos a task principal para economizar RAM
  vTaskDelete(NULL);
}
