// ESP32 LIDAR STREAMING v1.0 - Потоковая передача без буферизации

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

const char* ssid = "LEP";
const char* password = "Lep190972";

// UDP объекты
WiFiUDP cmdUdp;
WiFiUDP lidarUdp;

// UDP порты
const uint16_t CMD_PORT = 3333;
const uint16_t LIDAR_PORT = 3334;

// Динамический IP клиента
IPAddress client_ip;
bool has_client = false;

// LIDAR Serial
#define LIDAR_RX_PIN 12  // Желтый (TX лидара)
#define LIDAR_TX_PIN 11  // Зеленый (RX лидара)  
#define LIDAR_BAUDRATE 460800
HardwareSerial LidarSerial(1);

// Буферы команд
#define CMD_BUFFER_SIZE 256
char cmdBuffer[CMD_BUFFER_SIZE];

// Счетчики для статистики
unsigned long debugTimer = 0;
unsigned long lidarBytesRx = 0;
unsigned long lidarBytesTx = 0;
unsigned long cmdReceived = 0;
unsigned long streamPackets = 0;
bool firstLidarData = true;
bool lidarRunning = false;

// Буфер для потоковой передачи (очень маленький)
#define STREAM_BUFFER_SIZE 50
uint8_t streamBuffer[STREAM_BUFFER_SIZE];
int streamIndex = 0;
unsigned long lastStreamSend = 0;

void debugLog(String message) {
  Serial.printf("[%8lu] %s\n", millis(), message.c_str());
}

void testLidarConnection() {
  debugLog("🧪 === БЫСТРЫЙ ТЕСТ ЛИДАРА ===");
  
  delay(500);
  while (LidarSerial.available()) LidarSerial.read();
  
  // STOP
  debugLog("STOP...");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x25);
  LidarSerial.flush();
  delay(200);
  
  int bytes = 0;
  while (LidarSerial.available()) {
    LidarSerial.read();
    bytes++;
  }
  
  // RESET
  debugLog("RESET...");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x40);
  LidarSerial.flush();
  delay(1000);
  
  while (LidarSerial.available()) LidarSerial.read();
  
  // START SCAN
  debugLog("START SCAN...");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x20);
  LidarSerial.flush();
  delay(500);
  
  // Проверяем данные
  bytes = 0;
  unsigned long start = millis();
  while (millis() - start < 1000) {
    if (LidarSerial.available()) {
      LidarSerial.read();
      bytes++;
    }
  }
  
  int bytesPerSec = bytes;
  debugLog("Тест: " + String(bytesPerSec) + " байт/сек");
  
  if (bytesPerSec > 500) {
    debugLog("✅ LIDAR работает!");
    lidarRunning = true;
  } else {
    debugLog("❌ LIDAR не работает");
    lidarRunning = false;
  }
  
  debugLog("===================");
}

void setup() {
  Serial.begin(115200);
  
  delay(3000);
  
  Serial.println("ESP32 LIDAR STREAMING VERSION: 1.0");
  debugLog("╔════════════════════════════════════╗");
  debugLog("║    ESP32 LIDAR STREAMING v1.0      ║");
  debugLog("║  Потоковая передача без буферов    ║");
  debugLog("╚════════════════════════════════════╝");
  
  // LIDAR Serial (как в рабочем коде)
  debugLog("🔧 LIDAR Serial...");
  LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_TX_PIN, LIDAR_RX_PIN);
  debugLog("✅ LIDAR: RX=" + String(LIDAR_RX_PIN) + " TX=" + String(LIDAR_TX_PIN));
  
  // Быстрый тест
  testLidarConnection();
  
  // WiFi
  debugLog("📶 WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    debugLog("✅ WiFi: " + WiFi.localIP().toString());
  } else {
    debugLog("❌ WiFi не подключен!");
    return;
  }
  
  // UDP сокеты
  debugLog("🔧 UDP сокеты...");
  cmdUdp.begin(CMD_PORT);
  lidarUdp.begin(LIDAR_PORT);
  
  debugLog("✅ UDP готов:");
  debugLog("   Команды ← :" + String(CMD_PORT));
  debugLog("   LIDAR → :" + String(LIDAR_PORT));
  debugLog("   Режим: ПОТОКОВАЯ ПЕРЕДАЧА");
  
  // Финальный запуск лидара
  if (lidarRunning) {
    debugLog("🚀 Запуск потокового режима...");
    delay(200);
    LidarSerial.write(0xA5); LidarSerial.write(0x25); delay(100);
    LidarSerial.write(0xA5); LidarSerial.write(0x20);
    debugLog("✅ LIDAR в потоковом режиме!");
  }
  
  debugLog("╔════════════════════════════════════╗");
  debugLog("║           ПОТОКОВЫЙ РЕЖИМ          ║");
  debugLog("║    Каждый байт сразу передается    ║");
  debugLog("╚════════════════════════════════════╝");
  
  debugTimer = millis();
}

void handleCommands() {
  int packetSize = cmdUdp.parsePacket();
  if (packetSize > 0) {
    IPAddress sender_ip = cmdUdp.remoteIP();
    
    // Автоопределение IP клиента
    if (!has_client || client_ip != sender_ip) {
      client_ip = sender_ip;
      has_client = true;
      debugLog("🔗 КЛИЕНТ: " + client_ip.toString());
      debugLog("   Потоковая передача АКТИВНА!");
    }
    
    int len = cmdUdp.read(cmdBuffer, CMD_BUFFER_SIZE - 1);
    cmdReceived++;
    
    if (len > 0) {
      cmdBuffer[len] = '\0';
      
      if (cmdReceived <= 5) {
        debugLog("📥 #" + String(cmdReceived) + ": " + String(cmdBuffer));
      }
      
      // Команды для лидара
      if (len >= 2 && (uint8_t)cmdBuffer[0] == 0xA5) {
        debugLog("🎯 LIDAR команда: 0x" + String((uint8_t)cmdBuffer[1], HEX));
        for (int i = 0; i < len; i++) {
          LidarSerial.write((uint8_t)cmdBuffer[i]);
        }
        LidarSerial.flush();
      }
    }
  }
}

void streamLidarData() {
  // ПОТОКОВАЯ ПЕРЕДАЧА: читаем и сразу отправляем!
  
  // Читаем все доступные байты
  while (LidarSerial.available() && streamIndex < STREAM_BUFFER_SIZE - 1) {
    uint8_t byte = LidarSerial.read();
    streamBuffer[streamIndex++] = byte;
    lidarBytesRx++;
    
    if (firstLidarData) {
      debugLog("🎉 ПЕРВЫЙ БАЙТ LIDAR: 0x" + String(byte, HEX));
      firstLidarData = false;
    }
  }
  
  // Отправляем данные сразу если есть клиент
  if (streamIndex > 0 && has_client) {
    // ПОТОКОВАЯ ОТПРАВКА: отправляем каждые 2мс или при накоплении 10 байт
    if (streamIndex >= 10 || (millis() - lastStreamSend >= 2)) {
      
      int result = lidarUdp.beginPacket(client_ip, LIDAR_PORT);
      if (result) {
        size_t sent = lidarUdp.write(streamBuffer, streamIndex);
        result = lidarUdp.endPacket();
        
        if (result) {
          lidarBytesTx += sent;
          streamPackets++;
          
          // Логирование каждый 1000й пакет
          if (streamPackets % 1000 == 0) {
            debugLog("📤 Поток #" + String(streamPackets) + ": " + String(sent) + "B");
          }
        } else {
          // При ошибке просто сбрасываем буфер и продолжаем
          if (streamPackets % 100 == 0) {
            debugLog("⚠️ UDP ошибка (пакет " + String(streamPackets) + ")");
          }
        }
      }
      
      // Сбрасываем буфер для следующей порции
      streamIndex = 0;
      lastStreamSend = millis();
    }
  }
}

void printStreamStats() {
  if (millis() - debugTimer < 5000) return; // Каждые 5 сек
  
  unsigned long uptime = millis() / 1000;
  
  debugLog("╔═══════ ПОТОК СТАТИСТИКА ═══════╗");
  debugLog("║ Время: " + String(uptime) + "s");
  debugLog("║ WiFi: " + String(WiFi.status() == WL_CONNECTED ? "✅" : "❌") + " " + WiFi.localIP().toString());
  
  if (has_client) {
    debugLog("║ Клиент: ✅ " + client_ip.toString());
  } else {
    debugLog("║ Клиент: ⏳ Ожидание...");
  }
  
  debugLog("║");
  debugLog("║ 📊 ПОТОКОВЫЕ ДАННЫЕ:");
  
  // Скорость за последние 5 сек
  static unsigned long lastRxBytes = 0;
  unsigned long rxSpeed = (lidarBytesRx - lastRxBytes) / 5;
  lastRxBytes = lidarBytesRx;
  
  debugLog("║   Прием: " + String(rxSpeed) + " B/s");
  debugLog("║   Передано: " + String(lidarBytesTx) + " B");
  debugLog("║   Пакетов: " + String(streamPackets));
  debugLog("║   Команд: " + String(cmdReceived));
  
  if (rxSpeed > 2000) {
    debugLog("║ ✅ ОТЛИЧНАЯ СКОРОСТЬ!");
  } else if (rxSpeed > 500) {
    debugLog("║ ⚠️ Нормальная скорость");
  } else if (rxSpeed > 0) {
    debugLog("║ 🔶 Низкая скорость");
  } else if (uptime > 10) {
    debugLog("║ ❌ Нет данных LIDAR");
  }
  
  debugLog("╚════════════════════════════════╝");
  debugTimer = millis();
}

void loop() {
  handleCommands();    // Обработка команд
  streamLidarData();   // ПОТОКОВАЯ передача данных
  printStreamStats();  // Статистика
  
  yield(); // Для WiFi
}