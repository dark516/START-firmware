// ESP32 Simple UDP Echo Server
// Получает байт и отправляет его обратно

#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi настройки
const char* ssid = "s548-poligon";
const char* password = "Bwog4581";

// UDP настройки
WiFiUDP udp;
const uint16_t UDP_PORT = 3333;

// Статистика
unsigned long packetsReceived = 0;
unsigned long packetsSent = 0;
unsigned long lastStatsTime = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n╔═══════════════════════════════╗");
  Serial.println("║    ESP32 Simple Echo Server   ║");
  Serial.println("║         UDP Byte Test         ║");
  Serial.println("╚═══════════════════════════════╝");
  
  // WiFi подключение
  Serial.printf("\n📶 Подключение к WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts % 20 == 0) Serial.println();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ WiFi подключен!");
    Serial.printf("\n📍 IP адрес: %s", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n❌ Ошибка WiFi подключения!");
    return;
  }
  
  // UDP инициализация
  if (udp.begin(UDP_PORT)) {
    Serial.printf("\n✅ UDP сервер запущен на порту %d", UDP_PORT);
  } else {
    Serial.println("\n❌ Ошибка запуска UDP сервера!");
    return;
  }
  
  Serial.println("\n╔═══════════════════════════════╗");
  Serial.printf("║ ГОТОВ! Слушаем порт %-4d     ║\n", UDP_PORT);
  Serial.println("║ Отправляйте байты для теста  ║");
  Serial.println("╚═══════════════════════════════╝");
  
  lastStatsTime = millis();
}

void loop() {
  // Проверяем входящие UDP пакеты
  int packetSize = udp.parsePacket();
  
  if (packetSize > 0) {
    // Получаем данные
    uint8_t incomingByte;
    int bytesRead = udp.read(&incomingByte, 1);
    
    if (bytesRead == 1) {
      // Увеличиваем счетчик полученных пакетов
      packetsReceived++;
      
      // Получаем IP и порт отправителя
      IPAddress senderIP = udp.remoteIP();
      uint16_t senderPort = udp.remotePort();
      
      // Отправляем байт обратно
      udp.beginPacket(senderIP, senderPort);
      udp.write(&incomingByte, 1);
      
      if (udp.endPacket()) {
        packetsSent++;
        
        // Логируем успешный обмен (каждый 10-й)
        if (packetsReceived % 10 == 0) {
          Serial.printf("🏓 Echo #%lu: байт %d → %s:%d\n", 
                       packetsReceived, incomingByte, 
                       senderIP.toString().c_str(), senderPort);
        }
      } else {
        Serial.printf("❌ Ошибка отправки байта %d\n", incomingByte);
      }
    } else {
      Serial.printf("⚠️ Получено %d байт, ожидался 1\n", bytesRead);
    }
  }
  
  // Статистика каждые 10 секунд
  if (millis() - lastStatsTime > 10000) {
    printStats();
    lastStatsTime = millis();
  }
  
  yield(); // Даем время другим задачам
}

void printStats() {
  Serial.println("\n╔═══════════ СТАТИСТИКА ══════════╗");
  Serial.printf("║ WiFi: %s (%d dBm)      ║\n", 
                WiFi.status() == WL_CONNECTED ? "✅ OK" : "❌ ERR",
                WiFi.RSSI());
  Serial.printf("║ IP: %s     ║\n", WiFi.localIP().toString().c_str());
  Serial.println("╠═══════════════════════════════════╣");
  Serial.printf("║ Получено пакетов: %-6lu      ║\n", packetsReceived);
  Serial.printf("║ Отправлено пакетов: %-6lu    ║\n", packetsSent);
  
  if (packetsReceived > 0) {
    float successRate = (float)packetsSent / packetsReceived * 100;
    Serial.printf("║ Успешность: %.1f%%             ║\n", successRate);
  }
  
  Serial.println("╚═══════════════════════════════════╝");
}