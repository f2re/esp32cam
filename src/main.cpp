#include <Arduino.h>

// Определение пинов светодиодов
#define FLASH_LED 4   // GPIO4 - белая яркая вспышка камеры
#define RED_LED 33    // GPIO33 - красный индикаторный светодиод

void setup() {
  Serial.begin(115200);
  
  // Инициализация пинов светодиодов
  pinMode(FLASH_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  
  // Выключаем оба светодиода при старте
  digitalWrite(FLASH_LED, LOW);
  digitalWrite(RED_LED, HIGH);  // HIGH = выключен (инвертированная логика)
  
  Serial.println("ESP32-CAM запущен!");
  Serial.println("GPIO4 - вспышка, GPIO33 - красный LED");
}

void loop() {
  // Включаем вспышку на 1 секунду
//   digitalWrite(FLASH_LED, HIGH);
//   Serial.println("Вспышка ВКЛ");
//   delay(1000);
  
//   // Выключаем вспышку
//   digitalWrite(FLASH_LED, LOW);
//   Serial.println("Вспышка ВЫКЛ");
//   delay(1000);
  
  // Мигаем красным светодиодом
  digitalWrite(RED_LED, LOW);   // LOW = включен (инвертированная логика!)
  Serial.println("Красный LED ВКЛ");
  delay(1000);
  
  digitalWrite(RED_LED, HIGH);  // HIGH = выключен
  Serial.println("Красный LED ВЫКЛ");
  delay(1000);
}
