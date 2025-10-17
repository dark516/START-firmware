// ESP32 DEBUG версия с подробным логированием
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const char* ssid = "s548-poligon";
const char* password = "Bwog4581";

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

// === ОТЛАДОЧНЫЕ ФЛАГИ ===
#define DEBUG_VERBOSE 1
#define DEBUG_UDP 1
#define DEBUG_LIDAR 1
#define DEBUG_COMMANDS 1
#define DEBUG_SENSORS 1

// === БИНАРНАЯ СТРУКТУРА ДАТЧИКОВ ===
struct SensorDataPacket {
    uint32_t timestamp;     // millis()
    uint32_t left_ticks;    // Левый энкодер
    uint32_t right_ticks;   // Правый энкодер  
    uint16_t yaw_raw;       // yaw * 10 (0.1 град точность)
    uint16_t accel_raw;     // (accel + 10) * 1000 (0.001 м/с² точность)
    uint8_t packet_id;      // Счетчик пакетов 0-255
} __attribute__((packed));

static uint8_t sensor_packet_id = 0;

// UDP объекты
WiFiUDP cmdUdp;
WiFiUDP lidarUdp;
WiFiUDP sensorUdp;

// UDP порты
const uint16_t CMD_PORT = 3333;
const uint16_t LIDAR_PORT = 3334;
const uint16_t SENSOR_PORT = 3335;

// Динамический IP кёента
IPAddress client_ip;
bool has_client = false;

// LIDAR Serial
#define LIDAR_RX_PIN 12
#define LIDAR_TX_PIN 11
#define LIDAR_BAUDRATE 460800
HardwareSerial LidarSerial(1); 

// CAN
SPIClass mySpi = SPIClass(FSPI);
MCP2515 mcp2515(7);
struct can_frame canMsg;

// Буферы и счетчики
#define CMD_BUFFER_SIZE 256
#define LIDAR_BUFFER_SIZE 512   
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t lidarBuffer[LIDAR_BUFFER_SIZE];

unsigned long debugTimer = 0;
unsigned long lidarBytesRx = 0;
unsigned long lidarBytesTx = 0;
unsigned long sensorSendTimer = 0;
unsigned long cmdReceived = 0;
unsigned long lastDataTime = 0;
bool firstData = true;

const unsigned long SENSOR_SEND_INTERVAL = 50; // 20 Гц
const unsigned long DEBUG_INTERVAL = 5000;     // 5 сек

void debugPrint(const char* tag, const char* message) {
  #if DEBUG_VERBOSE
  Serial.printf("[%lu][%s] %s\n", millis(), tag, message);
  #endif
}

void debugPrintHex(const char* tag, uint8_t* data, int len) {
  #if DEBUG_VERBOSE
  Serial.printf("[%lu][%s] Hex: ", millis(), tag);
  for(int i = 0; i < len && i < 20; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.printf("(%d bytes)\n", len);
  #endif
}

int convertedX(int angle) {
  if (angle <= 180) return angle;
  return angle - 360;
}

void sendCommandToArduino(float linear_x, float angular_z) {
  struct can_frame cmdFrame;
  cmdFrame.can_id = 0x200;
  cmdFrame.can_dlc = 8;
  
  memcpy(&cmdFrame.data[0], &linear_x, 4);
  memcpy(&cmdFrame.data[4], &angular_z, 4);
  
  if (mcp2515.sendMessage(&cmdFrame) == MCP2515::ERROR_OK) {
    #if DEBUG_COMMANDS
    debugPrint("CAN", (String("→Arduino: ") + String(linear_x, 3) + " " + String(angular_z, 3)).c_str());
    #endif
  } else {
    debugPrint("CAN", "❌ Ошибка отправки CAN");
  }
}

void testLidar() {
  debugPrint("LIDAR", "🧪 Тест подключения LIDAR...");
  
  delay(500);
  while (LidarSerial.available()) LidarSerial.read();
  
  // STOP
  debugPrint("LIDAR", "Отправка STOP...");
  LidarSerial.write(0xA5); LidarSerial.write(0x25); LidarSerial.flush();
  delay(200);
  int bytes = 0;
  while (LidarSerial.available()) { LidarSerial.read(); bytes++; }
  debugPrint("LIDAR", (String("STOP ответ: ") + String(bytes) + " bytes").c_str());
  
  // START SCAN
  debugPrint("LIDAR", "Отправка START SCAN...");
  LidarSerial.write(0xA5); LidarSerial.write(0x20); LidarSerial.flush();
  delay(500);
  
  bytes = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (LidarSerial.available()) { 
      LidarSerial.read(); 
      bytes++; 
    }
  }
  
  int bytesPerSec = bytes / 2;
  debugPrint("LIDAR", (String("START ответ: ") + String(bytesPerSec) + " B/s").c_str());
  
  if (bytesPerSec > 1000) {
    debugPrint("LIDAR", "✅ LIDAR работает отлично!");
  } else if (bytesPerSec > 200) {
    debugPrint("LIDAR", "⚠️ LIDAR работает медленно");
  } else {
    debugPrint("LIDAR", "❌ LIDAR не отвечает");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  debugPrint("SYSTEM", "╔════════════════════════════════════╗");
  debugPrint("SYSTEM", "║  ESP32 DEBUG LIDAR Bridge         ║");
  debugPrint("SYSTEM", "║  Подробное логирование включено    ║");
  debugPrint("SYSTEM", "╚════════════════════════════════════╝");
  
  // Проверяем размер структуры
  debugPrint("STRUCT", (String("SensorDataPacket size: ") + String(sizeof(SensorDataPacket)) + " bytes").c_str());
  if (sizeof(SensorDataPacket) != 17) {
    debugPrint("STRUCT", "❌ КРИТИЧЕСКАЯ ОШИБКА: Неправильный размер структуры!");
    while(1) delay(1000);
  } else {
    debugPrint("STRUCT", "✅ Размер структуры корректен (17 байт)");
  }
  
  // LIDAR Serial
  debugPrint("LIDAR", "Инициализация LIDAR Serial...");
  LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN); 
  debugPrint("LIDAR", (String("✅ LIDAR: RX=") + String(LIDAR_RX_PIN) + " TX=" + String(LIDAR_TX_PIN) + " @ " + String(LIDAR_BAUDRATE) + " baud").c_str());
  
  // Тест LIDAR
  testLidar();
  
  // WiFi
  debugPrint("WIFI", (String("Подключение к ") + String(ssid) + "...").c_str());
  WiFi.begin(ssid, password);
  
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED && dots < 40) {
    delay(500);
    if (++dots % 10 == 0) debugPrint("WIFI", "Подключение...");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    debugPrint("WIFI", (String("✅ ESP32 IP: ") + WiFi.localIP().toString()).c_str());
  } else {
    debugPrint("WIFI", "❌ WiFi подключение не удалось!");
  }
  
  // UDP сокеты
  debugPrint("UDP", "Создание UDP сокетов...");
  cmdUdp.begin(CMD_PORT);
  lidarUdp.begin(LIDAR_PORT);
  sensorUdp.begin(SENSOR_PORT);
  
  debugPrint("UDP", "✅ UDP Сокеты созданы:");
  debugPrint("UDP", (String("   Команды ← :") + String(CMD_PORT)).c_str());
  debugPrint("UDP", (String("   LIDAR → :") + String(LIDAR_PORT)).c_str());
  debugPrint("UDP", (String("   Датчики → :") + String(SENSOR_PORT)).c_str());
  debugPrint("UDP", "   Client IP: Автоопределение");
  
  // CAN
  debugPrint("CAN", "Инициализация CAN...");
  mySpi.begin(4, 6, 5, 7);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  debugPrint("CAN", "✅ CAN готов");
  
  // IMU
  debugPrint("IMU", "Инициализация IMU...");
  Wire.begin(16, 15);
  if (bno.begin()) {
    bno.setMode(OPERATION_MODE_NDOF);
    debugPrint("IMU", "✅ BNO055 готов");
  } else {
    debugPrint("IMU", "❌ BNO055 не найден");
  }
  
  debugPrint("SYSTEM", "╔════════════════════════════════════╗");
  debugPrint("SYSTEM", (String("║ ГОТОВ! IP: ") + WiFi.localIP().toString() + " ║").c_str());
  debugPrint("SYSTEM", "║ Ожидание команд для регистрации...║");
  debugPrint("SYSTEM", "╚════════════════════════════════════╝");
  
  // Автозапуск LIDAR
  debugPrint("LIDAR", "⚙️ Автозапуск LIDAR...");
  delay(1000);
  LidarSerial.write(0xA5); LidarSerial.write(0x25); delay(100);  // STOP
  LidarSerial.write(0xA5); LidarSerial.write(0x20);             // START SCAN
  debugPrint("LIDAR", "✅ LIDAR автозапуск выполнен");
  
  debugTimer = millis();
}

void handleLidarUDP() {
  static int lidarBufferIndex = 0;
  static unsigned long lastSendTime = 0;
  const unsigned long SEND_INTERVAL = 20; // Отправлять каждые 20мс
  
  // Читаем данные от LIDAR
  int readBytes = 0;
  unsigned long readStart = millis();
  
  while (LidarSerial.available() && (millis() - readStart < 5) && lidarBufferIndex < LIDAR_BUFFER_SIZE) {
    uint8_t byte = LidarSerial.read();
    lidarBuffer[lidarBufferIndex++] = byte;
    lidarBytesRx++;
    readBytes++;
    
    if (firstData) {
      debugPrint("LIDAR", "🎉 ПЕРВЫЕ ДАННЫЕ LIDAR!");
      debugPrintHex("LIDAR", lidarBuffer, min(lidarBufferIndex, 10));
      firstData = false;
    }
    
    if (lidarBufferIndex >= LIDAR_BUFFER_SIZE - 50) {
      break;
    }
  }
  
  // Отправляем пакеты когда есть данные и клиент зарегистрирован
  bool shouldSend = false;
  
  if (lidarBufferIndex > 0 && has_client) {
    if (lidarBufferIndex >= 100 || (millis() - lastSendTime >= SEND_INTERVAL)) {
      shouldSend = true;
    }
  }
  
  if (shouldSend) {
    int sendSize = min(lidarBufferIndex, 300);  // Макс 300 байт за раз
    
    #if DEBUG_LIDAR
    static int packetCount = 0;
    packetCount++;
    if (packetCount <= 5 || packetCount % 50 == 0) {
      debugPrint("LIDAR", (String("📤 Отправка LIDAR #") + String(packetCount) + ": " + String(sendSize) + " байт → " + client_ip.toString()).c_str());
    }
    #endif
    
    int result = lidarUdp.beginPacket(client_ip, LIDAR_PORT);
    if (result) {
      size_t sent = lidarUdp.write(lidarBuffer, sendSize);
      result = lidarUdp.endPacket();
      
      if (result) {
        // Сдвигаем оставшиеся данные
        if (sendSize < lidarBufferIndex) {
          memmove(lidarBuffer, lidarBuffer + sendSize, lidarBufferIndex - sendSize);
          lidarBufferIndex -= sendSize;
        } else {
          lidarBufferIndex = 0;
        }
        lidarBytesTx += sent;
      } else {
        debugPrint("LIDAR", "⚠️ UDP endPacket failed");
        lidarBufferIndex = 0;
      }
    } else {
      debugPrint("LIDAR", "⚠️ UDP beginPacket failed");
      lidarBufferIndex = 0;
    }
    
    lastSendTime = millis();
    lastDataTime = millis();
  }
}

void handleCommandsUDP() {
  int packetSize = cmdUdp.parsePacket();
  if (packetSize > 0) {
    IPAddress sender_ip = cmdUdp.remoteIP();
    
    // АВТООПРЕДЕЛЕНИЕ IP КЛИЕНТА
    if (!has_client || client_ip != sender_ip) {
      client_ip = sender_ip;
      has_client = true;
      debugPrint("UDP", (String("🔗 Клиент зарегистрирован: ") + client_ip.toString() + " (автоопределение)").c_str());
    }
    
    int len = cmdUdp.read(cmdBuffer, CMD_BUFFER_SIZE - 1);
    cmdReceived++;
    
    if (len > 0) {
      #if DEBUG_COMMANDS
      if (cmdReceived <= 10 || cmdReceived % 20 == 0) {
        debugPrint("CMD", (String("📥 Команда #") + String(cmdReceived) + " от " + sender_ip.toString() + ": " + String(len) + " байт").c_str());
        debugPrintHex("CMD", (uint8_t*)cmdBuffer, len);
      }
      #endif
      
      // Проверяем тип команды
      bool isTextCommand = true;
      for (int i = 0; i < len; i++) {
        if (cmdBuffer[i] < 32 && cmdBuffer[i] != 10 && cmdBuffer[i] != 13) {
          isTextCommand = false;
          break;
        }
      }
      
      if (isTextCommand) {
        // Команды для двигателей
        cmdBuffer[len] = '\0';
        String command = String(cmdBuffer);
        command.trim();
        
        if (command.length() > 0) {
          int idx = command.indexOf(' ');
          if (idx > 0) {
            float lin = command.substring(0, idx).toFloat();
            float ang = command.substring(idx + 1).toFloat();
            sendCommandToArduino(lin, ang);
          } else {
            debugPrint("CMD", (String("Текстовая команда: '") + command + "'").c_str());
          }
        }
      } else {
        // Бинарные команды для LIDAR
        debugPrint("CMD", (String("Бинарная команда для LIDAR: ") + String(len) + " байт").c_str());
        for (int i = 0; i < len; i++) {
          LidarSerial.write((uint8_t)cmdBuffer[i]);
        }
      }
    }
  }
}

void sendSensorData() {
  if (millis() - sensorSendTimer < SENSOR_SEND_INTERVAL) {
    return;
  }
  
  // Читаем CAN сообщения от Arduino
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x100 && canMsg.can_dlc == 8) {
      int32_t left, right;
      memcpy(&left, &canMsg.data[0], 4);
      memcpy(&right, &canMsg.data[4], 4);
      
      // Читаем IMU
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      
      int yaw = convertedX((int)euler.x());
      float x_acc = accel.y();
      
      // ФОРМИРУЕМ БИНАРНЫЙ ПАКЕТ
      SensorDataPacket packet;
      packet.timestamp = millis();
      packet.left_ticks = (uint32_t)left;
      packet.right_ticks = (uint32_t)right;
      packet.yaw_raw = (uint16_t)((yaw + 360) * 10) % 3600;
      packet.accel_raw = (uint16_t)((x_acc + 10.0) * 1000);
      packet.packet_id = sensor_packet_id++;
      
      // Отправляем БИНАРНЫЙ пакет только если есть клиент
      if (has_client) {
        int result = sensorUdp.beginPacket(client_ip, SENSOR_PORT);
        if (result) {
          sensorUdp.write((uint8_t*)&packet, sizeof(packet));
          result = sensorUdp.endPacket();
          
          if (result) {
            #if DEBUG_SENSORS
            if (packet.packet_id <= 5 || packet.packet_id % 100 == 0) {
              debugPrint("SENSOR", (String("📦 Датчик #") + String(packet.packet_id) + ": L=" + String(packet.left_ticks) + " R=" + String(packet.right_ticks) + " Y=" + String(packet.yaw_raw/10.0, 1) + "°").c_str());
            }
            #endif
          } else {
            debugPrint("SENSOR", "⚠️ Sensor UDP send failed");
          }
        } else {
          debugPrint("SENSOR", "⚠️ Sensor UDP begin failed");
        }
      }
      
      sensorSendTimer = millis();
    }
  }
}

void printStats() {
  if (millis() - debugTimer < DEBUG_INTERVAL) return;
  
  debugPrint("STATS", "╔═════════ СТАТИСТИКА ══════════╗");
  debugPrint("STATS", (String("║ WiFi: ") + (WiFi.status() == WL_CONNECTED ? "✅" : "❌") + " (" + String(WiFi.RSSI()) + "dBm)").c_str());
  
  if (has_client) {
    debugPrint("STATS", (String("║ Клиент: ") + client_ip.toString()).c_str());
  } else {
    debugPrint("STATS", "║ Клиент: 🔍 Ожидание...");
  }
  
  debugPrint("STATS", "╠════════════════════════════════╣");
  
  static unsigned long lastRx = 0;
  unsigned long rxPerSec = (lidarBytesRx - lastRx) / (DEBUG_INTERVAL / 1000);
  lastRx = lidarBytesRx;
  
  debugPrint("STATS", (String("║ LIDAR RX: ") + String(rxPerSec) + " B/s").c_str());
  debugPrint("STATS", (String("║ LIDAR TX: ") + String(lidarBytesTx) + " B").c_str());
  debugPrint("STATS", (String("║ SENSOR: #") + String(sensor_packet_id) + " (Binary)").c_str());
  debugPrint("STATS", (String("║ КОМАНД: ") + String(cmdReceived)).c_str());
  
  if (lastDataTime > 0) {
    unsigned long ago = millis() - lastDataTime;
    if (ago < 1000) {
      debugPrint("STATS", "║ Статус: ✅ АКТИВЕН");
    } else {
      debugPrint("STATS", (String("║ Статус: ⚠️ ") + String(ago/1000) + "s назад").c_str());
    }
  }
  
  debugPrint("STATS", "╚════════════════════════════════╝");
  debugTimer = millis();
}

void loop() {
  handleCommandsUDP();
  handleLidarUDP();
  sendSensorData();
  printStats();
  
  yield();
}