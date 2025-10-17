// ESP32 ТОЛЬКО ЛИДАР + WiFi для тестирования
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

// LIDAR Serial (исправлено для ESP32-S3)
#define LIDAR_RX_PIN 12  // Желтый (TX лидара)
#define LIDAR_TX_PIN 11  // Зеленый (RX лидара)  
#define LIDAR_BAUDRATE 460800
HardwareSerial LidarSerial(1);

// Буферы
#define CMD_BUFFER_SIZE 256
#define LIDAR_BUFFER_SIZE 512
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t lidarBuffer[LIDAR_BUFFER_SIZE];

// Счетчики
unsigned long debugTimer = 0;
unsigned long lidarBytesRx = 0;
unsigned long lidarBytesTx = 0;
unsigned long cmdReceived = 0;
unsigned long lastLidarSend = 0;
bool firstLidarData = true;
bool lidarRunning = false;

void debugLog(String message) {
  Serial.printf("[%8lu] %s\n", millis(), message.c_str());
}

void testLidarConnection() {
  debugLog("🧪 === ТЕСТ ПОДКЛЮЧЕНИЯ ЛИДАРА (КОПИЯ ESP32-S3) ===");
  
  // Очистить буфер
  delay(500);
  while (LidarSerial.available()) LidarSerial.read();
  
  // 1. STOP команда
  debugLog("1️⃣ STOP...");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x25);
  LidarSerial.flush();
  delay(200);
  
  int bytes = 0;
  while (LidarSerial.available()) {
    LidarSerial.read();
    bytes++;
  }
  debugLog("   STOP ответ: " + String(bytes) + " байт");
  
  // 2. RESET команда (КАК В РАБОЧЕМ КОДЕ!)
  debugLog("2️⃣ RESET...");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x40);
  LidarSerial.flush();
  delay(2000); // Ждём перезагрузку
  
  while (LidarSerial.available()) {
    LidarSerial.read();
  }
  debugLog("   RESET выполнен");
  
  // 3. GET_HEALTH команда (КАК В РАБОЧЕМ КОДЕ!)
  debugLog("3️⃣ HEALTH...");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x52);
  LidarSerial.flush();
  delay(100);
  
  bytes = 0;
  String healthHex = "   Hex: ";
  unsigned long start = millis();
  while (millis() - start < 500) {
    if (LidarSerial.available()) {
      uint8_t b = LidarSerial.read();
      if (bytes < 10) {
        if (b < 0x10) healthHex += "0";
        healthHex += String(b, HEX) + " ";
      }
      bytes++;
    }
  }
  
  debugLog("   HEALTH ответ: " + String(bytes) + " байт");
  if (bytes > 0) {
    debugLog(healthHex);
  }
  
  if (bytes >= 7) {
    debugLog("✅ HEALTH Response OK!");
  } else if (bytes > 0) {
    debugLog("⚠️ HEALTH слабый отклик");
  }
  
  // 4. START SCAN команда
  debugLog("4️⃣ START SCAN...");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x20);
  LidarSerial.flush();
  delay(500);
  
  // Считаем данные в течение 2 секунд
  bytes = 0;
  start = millis();
  while (millis() - start < 2000) {
    if (LidarSerial.available()) {
      LidarSerial.read();
      bytes++;
    }
  }
  
  int bytesPerSec = bytes / 2;
  debugLog("   SCAN скорость: " + String(bytesPerSec) + " байт/сек");
  
  // Оценка как в рабочем коде
  if (bytesPerSec > 1000) {
    debugLog("✅ EXCELLENT! Full speed!");
    lidarRunning = true;
  } else if (bytesPerSec > 200) {
    debugLog("⚠️ Working but SLOW");
    debugLog("   Data loss possible!");
    lidarRunning = true;
  } else if (bytesPerSec > 0) {
    debugLog("⚠️ Very low speed");
    lidarRunning = false;
  } else {
    debugLog("❌ NO DATA!");
    debugLog("");
    debugLog("📌 Check:");
    debugLog("  - Power 5V 500mA+");
    debugLog("  - Yellow → GPIO" + String(LIDAR_RX_PIN));
    debugLog("  - Green  → GPIO" + String(LIDAR_TX_PIN));
    debugLog("  - GND connected");
    lidarRunning = false;
  }
  
  // STOP для очистки
  LidarSerial.write(0xA5);
  LidarSerial.write(0x25);
  delay(100);
  while (LidarSerial.available()) {
    LidarSerial.read();
  }
  
  debugLog("=====================================");
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  debugLog("╔════════════════════════════════════╗");
  debugLog("║    ESP32 LIDAR ONLY TEST           ║");
  debugLog("║   Только лидар + WiFi + отладка    ║");
  debugLog("╚════════════════════════════════════╝");
  
  // Инициализация LIDAR Serial (КАК В РАБОЧЕМ ESP32-S3!)
  debugLog("🔧 Инициализация LIDAR...");
  // ВАЖНО: В рабочем коде параметры RX и TX перевернуты!
  LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_TX_PIN, LIDAR_RX_PIN);  // 11, 12 как в esp32s3_FIXED.cpp
  debugLog("✅ LIDAR Serial: RX=" + String(LIDAR_RX_PIN) + " TX=" + String(LIDAR_TX_PIN) + " @ " + String(LIDAR_BAUDRATE) + " baud");
  debugLog("   (Инициализация с перевернутыми параметрами как в рабочем коде)");
  
  // Тестирование лидара
  testLidarConnection();
  
  // WiFi подключение
  debugLog("📶 Подключение к WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(1000);
    wifiAttempts++;
    if (wifiAttempts % 5 == 0) {
      debugLog("   Попытка " + String(wifiAttempts) + "/30...");
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    debugLog("✅ WiFi подключен!");
    debugLog("   IP адрес: " + WiFi.localIP().toString());
    debugLog("   Сигнал: " + String(WiFi.RSSI()) + " dBm");
  } else {
    debugLog("❌ WiFi подключение не удалось!");
    return;
  }
  
  // UDP сокеты
  debugLog("🔧 Создание UDP сокетов...");
  cmdUdp.begin(CMD_PORT);
  lidarUdp.begin(LIDAR_PORT);
  
  debugLog("✅ UDP сокеты готовы:");
  debugLog("   Команды ← порт " + String(CMD_PORT));
  debugLog("   LIDAR → порт " + String(LIDAR_PORT));
  debugLog("   Режим: Автоопределение IP клиента");
  
  // Финальный запуск лидара
  if (lidarRunning) {
    debugLog("🚀 Финальный запуск LIDAR в режиме сканирования...");
    delay(500);
    // STOP для очистки
    LidarSerial.write(0xA5); LidarSerial.write(0x25); 
    delay(200);
    // START SCAN
    LidarSerial.write(0xA5); LidarSerial.write(0x20);
    delay(200);
    debugLog("✅ LIDAR запущен и готов к передаче данных");
  }
  
  debugLog("╔════════════════════════════════════╗");
  debugLog("║           СИСТЕМА ГОТОВА!          ║");
  debugLog("║  Отправьте UDP команду на порт     ║");
  debugLog("║  " + String(CMD_PORT) + " для регистрации IP        ║");
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
      debugLog("🔗 КЛИЕНТ ЗАРЕГИСТРИРОВАН: " + client_ip.toString());
      debugLog("   Теперь будут отправляться данные LIDAR");
    }
    
    int len = cmdUdp.read(cmdBuffer, CMD_BUFFER_SIZE - 1);
    cmdReceived++;
    
    if (len > 0) {
      cmdBuffer[len] = '\0';
      
      // Показываем первые 10 команд подробно
      if (cmdReceived <= 10) {
        debugLog("📥 Команда #" + String(cmdReceived) + " от " + sender_ip.toString() + ": '" + String(cmdBuffer) + "' (" + String(len) + " байт)");
      }
      
      // Проверяем - это команда для лидара?
      bool isLidarCommand = false;
      if (len >= 2 && (uint8_t)cmdBuffer[0] == 0xA5) {
        isLidarCommand = true;
        debugLog("🎯 Команда для LIDAR: 0x" + String((uint8_t)cmdBuffer[0], HEX) + " 0x" + String((uint8_t)cmdBuffer[1], HEX));
        
        // Отправляем команду лидару
        for (int i = 0; i < len; i++) {
          LidarSerial.write((uint8_t)cmdBuffer[i]);
        }
        LidarSerial.flush();
      } else {
        // Обычная команда - игнорируем (не отправляем моторам)
        if (cmdReceived <= 10) {
          debugLog("ℹ️ Команда проигнорирована (не для лидара)");
        }
      }
    }
  }
}

void handleLidarData() {
  static int bufferIndex = 0;
  static unsigned long lastStats = 0;
  
  // Читаем данные от лидара
  int bytesRead = 0;
  unsigned long readStart = millis();
  
  // Читаем максимум 5мс или до заполнения буфера
  while (LidarSerial.available() && (millis() - readStart < 5) && bufferIndex < LIDAR_BUFFER_SIZE - 10) {
    uint8_t byte = LidarSerial.read();
    lidarBuffer[bufferIndex++] = byte;
    lidarBytesRx++;
    bytesRead++;
    
    // Показываем первые данные
    if (firstLidarData) {
      debugLog("🎉 ПЕРВЫЕ ДАННЫЕ LIDAR ПОЛУЧЕНЫ!");
      String hexStr = "   Первые 20 байт: ";
      for (int i = 0; i < min(20, bufferIndex); i++) {
        if (lidarBuffer[i] < 0x10) hexStr += "0";
        hexStr += String(lidarBuffer[i], HEX) + " ";
      }
      debugLog(hexStr);
      firstLidarData = false;
    }
  }
  
  // Отправляем данные если есть клиент и достаточно данных
  bool shouldSend = false;
  if (bufferIndex > 0 && has_client) {
    // ИСПРАВЛЕНИЕ: Отправляем реже и меньшими пакетами
    if (bufferIndex >= 100 || (millis() - lastLidarSend >= 25)) {
      shouldSend = true;
    }
  }
  
  if (shouldSend) {
    // ИСПРАВЛЕНИЕ: Еще меньше пакеты для стабильности UDP
    int sendSize = min(bufferIndex, 100);  // Максимум 100 байт
    
    // ИСПРАВЛЕНИЕ: Отправляем данные клиенту на его порт, НЕ на наш LIDAR_PORT!
    // Клиент слушает на своем порту 3334, а не на нашем
    int result = lidarUdp.beginPacket(client_ip, LIDAR_PORT);
    if (result) {
      size_t sent = lidarUdp.write(lidarBuffer, sendSize);
      result = lidarUdp.endPacket();
      
      if (result) {
        lidarBytesTx += sent;
        
        // Сдвигаем оставшиеся данные
        if (sendSize < bufferIndex) {
          memmove(lidarBuffer, lidarBuffer + sendSize, bufferIndex - sendSize);
          bufferIndex -= sendSize;
        } else {
          bufferIndex = 0;
        }
        
        // Статистика отправки каждые 3 секунды
        if (millis() - lastStats > 3000) {
          debugLog("📤 LIDAR передача: отправлено " + String(sent) + " байт → " + client_ip.toString());
          lastStats = millis();
        }
        
      } else {
        debugLog("⚠️ UDP endPacket ошибка");
        bufferIndex = 0;
      }
    } else {
      debugLog("⚠️ UDP beginPacket ошибка");  
      bufferIndex = 0;
    }
    
    lastLidarSend = millis();
  }
}

void printDetailedStats() {
  if (millis() - debugTimer < 8000) return; // Каждые 8 секунд
  
  unsigned long uptime = millis() / 1000;
  
  debugLog("╔═══════════ СТАТИСТИКА ════════════╗");
  debugLog("║ Время работы: " + String(uptime) + " сек");
  debugLog("║ WiFi: " + String(WiFi.status() == WL_CONNECTED ? "✅ " : "❌ ") + WiFi.localIP().toString() + " (" + String(WiFi.RSSI()) + " dBm)");
  
  if (has_client) {
    debugLog("║ Клиент: ✅ " + client_ip.toString());
  } else {
    debugLog("║ Клиент: ⏳ Ожидание регистрации...");
  }
  
  debugLog("║");
  debugLog("║ 📊 LIDAR СТАТИСТИКА:");
  debugLog("║   Статус: " + String(lidarRunning ? "✅ Работает" : "❌ Не работает"));
  
  // Скорость приема за последние 8 сек
  static unsigned long lastRxBytes = 0;
  unsigned long rxSpeed = (lidarBytesRx - lastRxBytes) / 8;
  lastRxBytes = lidarBytesRx;
  debugLog("║   Прием: " + String(rxSpeed) + " байт/сек");
  
  debugLog("║   Всего получено: " + String(lidarBytesRx) + " байт");
  debugLog("║   Всего отправлено: " + String(lidarBytesTx) + " байт");
  
  debugLog("║");
  debugLog("║ 📨 КОМАНДЫ:");
  debugLog("║   Получено: " + String(cmdReceived));
  
  // Диагностика
  if (lidarBytesRx == 0 && uptime > 15) {
    debugLog("║ ⚠️ ВНИМАНИЕ: LIDAR не передает данные!");
  } else if (rxSpeed < 500 && lidarRunning && uptime > 15) {
    debugLog("║ ⚠️ ВНИМАНИЕ: Низкая скорость LIDAR");  
  } else if (rxSpeed > 1000) {
    debugLog("║ ✅ LIDAR работает отлично!");
  }
  
  if (!has_client && uptime > 10) {
    debugLog("║ ⚠️ Клиент не зарегистрирован - отправьте UDP команду");
  }
  
  debugLog("╚═══════════════════════════════════╝");
  debugTimer = millis();
}

void loop() {
  handleCommands();
  handleLidarData(); 
  printDetailedStats();
  
  yield(); // Позволяем WiFi работать
}