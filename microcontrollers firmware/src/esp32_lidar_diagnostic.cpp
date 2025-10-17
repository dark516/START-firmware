// ESP32 РАСШИРЕННАЯ ДИАГНОСТИКА ЛИДАРА
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

const char* ssid = "s548-poligon";
const char* password = "Bwog4581";

// UDP для связи
WiFiUDP cmdUdp;
IPAddress client_ip;
bool has_client = false;
const uint16_t CMD_PORT = 3333;

// LIDAR конфигурации для тестирования
struct LidarConfig {
  int rx_pin;

   int tx_pin;
  int baudrate;
  String description;
};

LidarConfig configs[] = {
  {12, 11, 460800, "Основная (RX=12, TX=11, 460800)"},
  {11, 12, 460800, "Обратная (RX=11, TX=12, 460800)"},
  {12, 11, 115200, "Низкая скорость (RX=12, TX=11, 115200)"},
  {12, 11, 256000, "Средняя скорость (RX=12, TX=11, 256000)"},
  {16, 17, 460800, "Альтернативные пины (RX=16, TX=17, 460800)"},
  {2, 4, 460800, "GPIO 2,4 (RX=2, TX=4, 460800)"}
};

HardwareSerial LidarSerial(1);
int currentConfig = 0;

void debugLog(String message) {
  Serial.printf("[%8lu] %s\n", millis(), message.c_str());
}

void setupWiFi() {
  debugLog("📶 Подключение к WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
    if (attempts % 4 == 0) debugLog("   Попытка " + String(attempts) + "/20...");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    debugLog("✅ WiFi: " + WiFi.localIP().toString() + " (сигнал: " + String(WiFi.RSSI()) + " dBm)");
    cmdUdp.begin(CMD_PORT);
  } else {
    debugLog("❌ WiFi не подключен!");
  }
}

void testLidarConfig(LidarConfig& config) {
  debugLog("╔═══════════════════════════════════════════╗");
  debugLog("║ ТЕСТ: " + config.description);
  debugLog("╚═══════════════════════════════════════════╝");
  
  // Инициализация Serial с текущей конфигурацией
  LidarSerial.end();
  delay(100);
  LidarSerial.begin(config.baudrate, SERIAL_8N1, config.rx_pin, config.tx_pin);
  delay(500);
  
  // Очищаем буфер
  while (LidarSerial.available()) LidarSerial.read();
  debugLog("📡 Serial настроен: RX=" + String(config.rx_pin) + " TX=" + String(config.tx_pin) + " @ " + String(config.baudrate) + " baud");
  
  // Тест 1: Проверка на любые данные
  debugLog("🔍 Тест 1: Поиск любых данных (5 сек)...");
  int randomBytes = 0;
  unsigned long testStart = millis();
  while (millis() - testStart < 5000) {
    if (LidarSerial.available()) {
      uint8_t byte = LidarSerial.read();
      randomBytes++;
      
      // Показываем первые 20 байт
      if (randomBytes <= 20) {
        debugLog("   Байт #" + String(randomBytes) + ": 0x" + String(byte, HEX) + " (" + String(byte) + ")");
      }
    }
    delay(1);
  }
  
  if (randomBytes > 0) {
    debugLog("✅ НАЙДЕНЫ ДАННЫЕ! Всего: " + String(randomBytes) + " байт за 5 сек");
    debugLog("   Скорость: " + String(randomBytes / 5) + " байт/сек");
  } else {
    debugLog("❌ Нет данных от лидара");
  }
  
  // Тест 2: Команда RESET
  debugLog("🔍 Тест 2: RESET команда...");
  while (LidarSerial.available()) LidarSerial.read(); // Очистка
  
  LidarSerial.write(0xA5);
  LidarSerial.write(0x40); // RESET
  LidarSerial.flush();
  delay(1000);
  
  int resetResponse = 0;
  testStart = millis();
  while (millis() - testStart < 2000) {
    if (LidarSerial.available()) {
      LidarSerial.read();
      resetResponse++;
    }
  }
  debugLog("   RESET ответ: " + String(resetResponse) + " байт");
  
  // Тест 3: Команда GET_INFO
  debugLog("🔍 Тест 3: GET_INFO команда...");
  while (LidarSerial.available()) LidarSerial.read(); // Очистка
  
  LidarSerial.write(0xA5);
  LidarSerial.write(0x50); // GET_INFO
  LidarSerial.flush();
  delay(500);
  
  int infoResponse = 0;
  uint8_t infoBytes[20];
  testStart = millis();
  while (millis() - testStart < 2000 && infoResponse < 20) {
    if (LidarSerial.available()) {
      infoBytes[infoResponse] = LidarSerial.read();
      infoResponse++;
    }
  }
  
  debugLog("   GET_INFO ответ: " + String(infoResponse) + " байт");
  if (infoResponse > 0) {
    String hexStr = "   Hex: ";
    for (int i = 0; i < min(infoResponse, 20); i++) {
      if (infoBytes[i] < 0x10) hexStr += "0";
      hexStr += String(infoBytes[i], HEX) + " ";
    }
    debugLog(hexStr);
  }
  
  // Тест 4: Команда START SCAN
  debugLog("🔍 Тест 4: START SCAN команда...");
  while (LidarSerial.available()) LidarSerial.read(); // Очистка
  
  LidarSerial.write(0xA5);
  LidarSerial.write(0x20); // START SCAN
  LidarSerial.flush();
  delay(1000);
  
  int scanBytes = 0;
  uint8_t scanData[50];
  testStart = millis();
  while (millis() - testStart < 3000 && scanBytes < 50) {
    if (LidarSerial.available()) {
      scanData[scanBytes] = LidarSerial.read();
      scanBytes++;
    }
  }
  
  debugLog("   START SCAN ответ: " + String(scanBytes) + " байт за 3 сек");
  if (scanBytes > 0) {
    String hexStr = "   Первые байты: ";
    for (int i = 0; i < min(scanBytes, 30); i++) {
      if (scanData[i] < 0x10) hexStr += "0";
      hexStr += String(scanData[i], HEX) + " ";
      if ((i + 1) % 10 == 0) hexStr += "\n   ";
    }
    debugLog(hexStr);
    
    debugLog("   Скорость сканирования: " + String(scanBytes / 3) + " байт/сек");
  }
  
  // Итоговая оценка
  int totalBytes = randomBytes + resetResponse + infoResponse + scanBytes;
  debugLog("📊 ИТОГО для этой конфигурации: " + String(totalBytes) + " байт");
  
  if (scanBytes > 100) {
    debugLog("🏆 ОТЛИЧНО! Лидар работает с этой конфигурацией!");
  } else if (totalBytes > 50) {
    debugLog("⚠️ Есть ответы, но не полные");
  } else if (totalBytes > 0) {
    debugLog("🔶 Слабый сигнал");
  } else {
    debugLog("❌ Нет связи с лидаром");
  }
  
  debugLog("");
  delay(2000);
}

void handleCommands() {
  int packetSize = cmdUdp.parsePacket();
  if (packetSize > 0) {
    IPAddress sender_ip = cmdUdp.remoteIP();
    
    if (!has_client || client_ip != sender_ip) {
      client_ip = sender_ip;
      has_client = true;
      debugLog("🔗 Клиент зарегистрирован: " + client_ip.toString());
    }
    
    char cmdBuffer[256];
    int len = cmdUdp.read(cmdBuffer, sizeof(cmdBuffer) - 1);
    if (len > 0) {
      cmdBuffer[len] = '\0';
      String cmd = String(cmdBuffer).trim();
      
      debugLog("📥 Команда: '" + cmd + "'");
      
      if (cmd == "NEXT_CONFIG") {
        currentConfig = (currentConfig + 1) % (sizeof(configs) / sizeof(configs[0]));
        debugLog("🔄 Переключение на конфигурацию #" + String(currentConfig));
        testLidarConfig(configs[currentConfig]);
      } else if (cmd == "RETEST") {
        debugLog("🔄 Повторное тестирование текущей конфигурации");
        testLidarConfig(configs[currentConfig]);
      } else if (cmd == "STATUS") {
        debugLog("📊 Текущая конфигурация #" + String(currentConfig) + ": " + configs[currentConfig].description);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  debugLog("╔════════════════════════════════════╗");
  debugLog("║       ESP32 LIDAR DIAGNOSTIC       ║");
  debugLog("║    Расширенное тестирование        ║");
  debugLog("╚════════════════════════════════════╝");
  
  setupWiFi();
  
  debugLog("🔧 Начинаем диагностику лидара...");
  debugLog("   Всего конфигураций: " + String(sizeof(configs) / sizeof(configs[0])));
  
  // Тестируем все конфигурации по очереди
  for (int i = 0; i < sizeof(configs) / sizeof(configs[0]); i++) {
    currentConfig = i;
    testLidarConfig(configs[i]);
    delay(1000);
  }
  
  debugLog("╔════════════════════════════════════╗");
  debugLog("║        ДИАГНОСТИКА ЗАВЕРШЕНА       ║");
  debugLog("║                                    ║");
  debugLog("║ ROS команды:                       ║");
  debugLog("║ - NEXT_CONFIG (следующий тест)     ║");
  debugLog("║ - RETEST (повтор текущего)         ║");
  debugLog("║ - STATUS (текущий статус)          ║");
  debugLog("╚════════════════════════════════════╝");
}

void loop() {
  handleCommands();
  
  // Периодически выводим напоминание
  static unsigned long lastReminder = 0;
  if (millis() - lastReminder > 30000) {
    debugLog("💡 Отправьте 'NEXT_CONFIG' для следующего теста");
    lastReminder = millis();
  }
  
  delay(100);
}