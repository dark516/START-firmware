// esp32s3_rplidar_bridge_FIXED.cpp - С БИНАРНЫМИ ПАКЕТАМИ ДАТЧИКОВ
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

// === БИНАРНАЯ СТРУКТУРА ДАТЧИКОВ (НОВОЕ!) ===
// Соответствует Python формату '<LLLHHB' = 17 байт
struct SensorDataPacket {
    uint32_t timestamp;     // millis()
    uint32_t left_ticks;    // Левый энкодер
    uint32_t right_ticks;   // Правый энкодер  
    uint16_t yaw_raw;       // yaw * 10 (0.1 град точность)
    uint16_t accel_raw;     // (accel + 10) * 1000 (0.001 м/с² точность)
    uint8_t packet_id;      // Счетчик пакетов 0-255
} __attribute__((packed));  // Важно: убираем выравнивание!

static uint8_t sensor_packet_id = 0;

// UDP объекты
WiFiUDP cmdUdp;
WiFiUDP lidarUdp;
WiFiUDP sensorUdp;

// UDP порты
const uint16_t CMD_PORT = 3333;       // Порт для приема команд
const uint16_t LIDAR_PORT = 3334;     // Порт для передачи LIDAR данных
const uint16_t SENSOR_PORT = 3335;    // Порт для передачи данных датчиков

// Динамические адреса клиентов (определяются автоматически)
IPAddress last_cmd_ip;     
IPAddress last_sensor_ip;  
bool has_cmd_client = false;
bool has_sensor_client = false;

// === RPLIDAR НА ПРОГРАММНОМ UART ===
#define LIDAR_RX_PIN 12  // Желтый (TX лидара)
#define LIDAR_TX_PIN 11  // Зеленый (RX лидара)
#define LIDAR_BAUDRATE 460800  

HardwareSerial LidarSerial(1); 

SPIClass mySpi = SPIClass(FSPI);
MCP2515 mcp2515(7);
struct can_frame canMsg;

// UDP буферы (уменьшенные для стабильности)
#define CMD_BUFFER_SIZE 256
#define LIDAR_BUFFER_SIZE 512   
#define SENSOR_BUFFER_SIZE 128

char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t lidarBuffer[LIDAR_BUFFER_SIZE];

// Счетчики и таймеры
unsigned long debugTimer = 0;
unsigned long lidarBytesRx = 0;
unsigned long lidarBytesTx = 0;
unsigned long sensorSendTimer = 0;
unsigned long lastDataTime = 0;
bool firstData = true;

// Частота отправки данных датчиков (мс)
const unsigned long SENSOR_SEND_INTERVAL = 20;  // 50 Гц

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
    Serial.printf("→Arduino: %.2f %.2f\n", linear_x, angular_z);
  }
}

// === ТЕСТ ЛИДАРА ===
void testLidar() {
  Serial.println("\n╔═══════════════════════════════╗");
  Serial.println("║  RPLIDAR C1 CONNECTION TEST   ║");
  Serial.println("╚═══════════════════════════════╝");
  Serial.printf("HardwareSerial: RX=%d TX=%d %d baud\n", 
                LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUDRATE);
  
  delay(500);
  
  // Очистка
  while (LidarSerial.available()) {
    LidarSerial.read();
  }
  
  // 1. STOP
  Serial.print("\n1️⃣ STOP... ");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x25);
  LidarSerial.flush();
  delay(200);
  
  int bytes = 0;
  while (LidarSerial.available()) {
    LidarSerial.read();
    bytes++;
  }
  Serial.printf("(%d bytes)\n", bytes);
  
  // 2. RESET
  Serial.print("2️⃣ RESET... ");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x40);
  LidarSerial.flush();
  delay(2000); // Ждём перезагрузку
  
  while (LidarSerial.available()) {
    LidarSerial.read();
  }
  Serial.println("OK");
  
  // 3. GET_HEALTH
  Serial.print("3️⃣ HEALTH... ");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x52);
  LidarSerial.flush();
  delay(100);
  
  bytes = 0;
  unsigned long start = millis();
  while (millis() - start < 500) {
    if (LidarSerial.available()) {
      uint8_t b = LidarSerial.read();
      if (bytes < 10) {
        Serial.printf("%02X ", b);
      }
      bytes++;
    }
  }
  Serial.printf("(%d bytes)\n", bytes);
  
  if (bytes >= 7) {
    Serial.println("✅ Response OK!");
  } else {
    Serial.println("⚠️ Weak response");
  }
  
  // 4. START SCAN
  Serial.print("\n4️⃣ SCAN... ");
  LidarSerial.write(0xA5);
  LidarSerial.write(0x20);
  LidarSerial.flush();
  delay(500);
  
  bytes = 0;
  start = millis();
  while (millis() - start < 2000) {
    if (LidarSerial.available()) {
      LidarSerial.read();
      bytes++;
    }
  }
  
  int bytesPerSec = bytes / 2;
  Serial.printf("%d B/s\n", bytesPerSec);
  
  if (bytesPerSec > 1000) {
    Serial.println("✅ EXCELLENT! Full speed!");
  } else if (bytesPerSec > 200) {
    Serial.println("⚠️ Working but SLOW");
    Serial.println("   Data loss possible!");
  } else if (bytesPerSec > 0) {
    Serial.println("⚠️ Very low speed");
  } else {
    Serial.println("❌ NO DATA!");
    Serial.println("\n📌 Check:");
    Serial.println("  - Power 5V 500mA+");
    Serial.printf("  - Yellow → GPIO%d\n", LIDAR_RX_PIN);
    Serial.printf("  - Green  → GPIO%d\n", LIDAR_TX_PIN);
    Serial.println("  - GND connected");
  }
  
  // STOP
  LidarSerial.write(0xA5);
  LidarSerial.write(0x25);
  delay(100);
  
  while (LidarSerial.available()) {
    LidarSerial.read();
  }
  
  Serial.println("═══════════════════════════════\n");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  ESP32 RPLidar C1 WiFi Bridge     ║");
  Serial.println("║   FIXED: Binary Sensor Packets    ║");
  Serial.println("╚════════════════════════════════════╝");
  
  // Проверяем размер структуры (КРИТИЧНО!)
  Serial.printf("\n🔧 SensorDataPacket size: %d bytes (expected: 17)\n", sizeof(SensorDataPacket));
  if (sizeof(SensorDataPacket) != 17) {
    Serial.println("❌ КРИТИЧЕСКАЯ ОШИБКА: Неправильный размер структуры!");
    Serial.println("   Добавьте __attribute__((packed)) к структуре");
    Serial.println("   Или проверьте настройки компилятора");
    while(1) delay(1000); // Останавливаемся
  } else {
    Serial.println("✅ Размер структуры корректен (17 байт)");
  }
  
  // === HARDWARE SERIAL ===
  Serial.println("\n🔧 Init HardwareSerial...");
  LidarSerial.begin(460800, SERIAL_8N1, 11, 12); 
  
  Serial.printf("✅ HardwareSerial: RX=%d TX=%d @ %d baud\n", 
                LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUDRATE);
  
  // === ТЕСТ ===
  testLidar();
  
  // === WiFi ===
  Serial.printf("\n📶 WiFi: %s", ssid);
  WiFi.begin(ssid, password);
  
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED && dots < 40) {
    delay(500);
    Serial.print(".");
    if (++dots % 20 == 0) Serial.println();
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n❌ WiFi failed!");
  }
  
  // === UDP инициализация ===
  cmdUdp.begin(CMD_PORT);
  lidarUdp.begin(LIDAR_PORT);
  sensorUdp.begin(SENSOR_PORT);
  
  Serial.println("\n✅ UDP Sockets:");
  Serial.printf("   :%d - Commands (receive)\n", CMD_PORT);
  Serial.printf("   :%d - LIDAR (send)\n", LIDAR_PORT);
  Serial.printf("   :%d - Sensors (send BINARY)\n", SENSOR_PORT);
  Serial.println("   ROS2 IP: Автоопределение по первой команде");
  
  // === CAN ===
  Serial.print("\n🔧 CAN... ");
  mySpi.begin(4, 6, 5, 7);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("OK");
  
  // === IMU ===
  Serial.print("🧭 IMU... ");
  Wire.begin(16, 15);
  if (bno.begin()) {
    bno.setMode(OPERATION_MODE_NDOF);
    Serial.println("OK");
  } else {
    Serial.println("NOT FOUND");
  }
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.printf("║ READY! IP: %s ║\n", WiFi.localIP().toString().c_str());
  Serial.println("╚════════════════════════════════════╝");
  
  // === Автоматический запуск LIDAR ===
  Serial.println("⚙️ Автозапуск LIDAR...");
  delay(1000);
  // STOP
  LidarSerial.write(0xA5);
  LidarSerial.write(0x25);
  delay(100);
  // START SCAN
  LidarSerial.write(0xA5);
  LidarSerial.write(0x20);
  Serial.println("✅ LIDAR автозапуск выполнен");
  
  debugTimer = millis();
}

// === UDP LIDAR МОСТ ===
void handleLidarUDP() {
  static int lidarBufferIndex = 0;
  static unsigned long lastSendTime = 0;
  const unsigned long SEND_INTERVAL = 10; // Отправлять каждые 10мс
  
  // Читаем данные от LIDAR
  int readBytes = 0;
  unsigned long readStart = millis();
  
  // Читаем данные порциями
  while (LidarSerial.available() && (millis() - readStart < 3) && lidarBufferIndex < LIDAR_BUFFER_SIZE) {
    uint8_t byte = LidarSerial.read();
    lidarBuffer[lidarBufferIndex++] = byte;
    lidarBytesRx++;
    readBytes++;
    
    if (firstData) {
      Serial.println("🎉 FIRST LIDAR DATA!");
      firstData = false;
    }
    
    // Отправляем пакет, если буфер полон
    if (lidarBufferIndex >= LIDAR_BUFFER_SIZE - 50) {
      break;
    }
  }
  
  // Отправляем маленькие UDP пакеты чаще
  bool shouldSend = false;
  
  if (lidarBufferIndex > 0) {
    // Отправляем маленькие пакеты (200 байт) чаще
    if (lidarBufferIndex >= 200 || 
        (millis() - lastSendTime >= SEND_INTERVAL)) {
      shouldSend = true;
    }
  }
  
  if (shouldSend && has_cmd_client) {
    // Ограничиваем размер пакета для надежности
    int sendSize = min(lidarBufferIndex, 400);  // Макс 400 байт за раз
    
    int result = lidarUdp.beginPacket(last_cmd_ip, LIDAR_PORT);
    if (result) {
      size_t sent = lidarUdp.write(lidarBuffer, sendSize);
      result = lidarUdp.endPacket();
      
      if (result) {
        // Успешно отправлено
        static unsigned long totalBytesSent = 0;
        totalBytesSent += sent;
        
        // Обновляем статистику каждую секунду
        static unsigned long lastStatsUpdate = 0;
        if (millis() - lastStatsUpdate > 1000) {
          lidarBytesTx = totalBytesSent;
          totalBytesSent = 0;
          lastStatsUpdate = millis();
        }
        
        // Сдвигаем оставшиеся данные
        if (sendSize < lidarBufferIndex) {
          memmove(lidarBuffer, lidarBuffer + sendSize, lidarBufferIndex - sendSize);
          lidarBufferIndex -= sendSize;
        } else {
          lidarBufferIndex = 0;
        }
        
      } else {
        // Ошибка отправки - очищаем буфер
        Serial.println("⚠️ LIDAR UDP send failed");
        lidarBufferIndex = 0;  // Очищаем при ошибке
      }
    } else {
      Serial.println("⚠️ LIDAR UDP begin failed");
      lidarBufferIndex = 0;
    }
    
    lastSendTime = millis();
    lastDataTime = millis();
  }
}

// === UDP КОМАНДЫ ОТ ROS2 ===
void handleCommandsUDP() {
  int packetSize = cmdUdp.parsePacket();
  if (packetSize > 0) {
    // Запоминаем IP отправителя
    IPAddress client_ip = cmdUdp.remoteIP();
    if (!has_cmd_client || client_ip != last_cmd_ip) {
      last_cmd_ip = client_ip;
      has_cmd_client = true;
      Serial.printf("🔗 Новый клиент команд: %s\n", client_ip.toString().c_str());
    }
    
    int len = cmdUdp.read(cmdBuffer, CMD_BUFFER_SIZE - 1);
    
    if (len > 0) {
      // Проверяем, это текстовая команда для двигателей или бинарные данные для LIDAR
      bool isTextCommand = true;
      for (int i = 0; i < len; i++) {
        if (cmdBuffer[i] < 32 && cmdBuffer[i] != 10 && cmdBuffer[i] != 13) {
          isTextCommand = false;
          break;
        }
      }
      
      if (isTextCommand) {
        // Команды для двигателей ("linear angular")
        cmdBuffer[len] = '\0';
        String command = String(cmdBuffer);
        command.trim();
        
        if (command.length() > 0) {
          int idx = command.indexOf(' ');
          if (idx > 0) {
            float lin = command.substring(0, idx).toFloat();
            float ang = command.substring(idx + 1).toFloat();
            sendCommandToArduino(lin, ang);
            Serial.printf("→Ардуино: %.3f %.3f\n", lin, ang);
          }
        }
      } else {
        // Бинарные команды для LIDAR
        for (int i = 0; i < len; i++) {
          LidarSerial.write((uint8_t)cmdBuffer[i]);
        }
        lidarBytesTx += len;
        Serial.printf("→LIDAR: %d bytes\n", len);
      }
    }
  }
}

// === ОТПРАВКА БИНАРНЫХ ДАННЫХ ДАТЧИКОВ (ИСПРАВЛЕНО!) ===
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
      
      // === ФОРМИРУЕМ БИНАРНЫЙ ПАКЕТ ===
      SensorDataPacket packet;
      packet.timestamp = millis();
      packet.left_ticks = (uint32_t)left;
      packet.right_ticks = (uint32_t)right;
      packet.yaw_raw = (uint16_t)((yaw + 360) * 10) % 3600;  // 0-3599 (0.1 град точность)
      packet.accel_raw = (uint16_t)((x_acc + 10.0) * 1000);  // -10 to +10 м/с² (0.001 точность)
      packet.packet_id = sensor_packet_id++;
      
      // Отправляем БИНАРНЫЙ пакет
      if (has_cmd_client) {
        int result = sensorUdp.beginPacket(last_cmd_ip, SENSOR_PORT);
        if (result) {
          sensorUdp.write((uint8_t*)&packet, sizeof(packet));
          result = sensorUdp.endPacket();
          
          if (result) {
            // Логирование каждые 50 пакетов
            if (packet.packet_id % 50 == 0) {
              Serial.printf("📦 Sensor #%d: L=%lu R=%lu Y=%.1f A=%.3f\n", 
                           packet.packet_id, packet.left_ticks, packet.right_ticks,
                           packet.yaw_raw / 10.0, (packet.accel_raw / 1000.0) - 10.0);
            }
          } else {
            Serial.println("⚠️ Sensor UDP send failed");
          }
        }
      }
      
      sensorSendTimer = millis();
    }
  }
}

// === СТАТИСТИКА ===
void printStats() {
  if (millis() - debugTimer < 10000) return;
  
  Serial.println("\n╔═════════ UDP STATUS ══════════╗");
  Serial.printf("║ WiFi: %s (%ddBm)     ║\n", 
                WiFi.status() == WL_CONNECTED ? "✅" : "❌",
                WiFi.RSSI());
  
  if (has_cmd_client) {
    Serial.printf("║ ROS2: %s ║\n", last_cmd_ip.toString().c_str());
  } else {
    Serial.println("║ ROS2: 🔍 Ожидание клиента     ║");
  }
  
  Serial.println("╠════════════════════════════════╣");
  
  static unsigned long lastRx = 0;
  unsigned long rxPerSec = (lidarBytesRx - lastRx) / 10;  // За 10 секунд
  lastRx = lidarBytesRx;
  
  Serial.printf("║ LIDAR RX: %6lu B/s        ║\n", rxPerSec);
  Serial.printf("║ LIDAR TX: %6lu B          ║\n", lidarBytesTx);
  Serial.printf("║ SENSOR: #%-6d (Binary)   ║\n", sensor_packet_id);
  
  if (lastDataTime > 0) {
    unsigned long ago = millis() - lastDataTime;
    if (ago < 1000) {
      Serial.println("║ Status: ✅ ACTIVE             ║");
    } else {
      Serial.printf("║ Status: ⚠️ %4lus ago         ║\n", ago/1000);
    }
  }
  
  Serial.println("╚════════════════════════════════╝");
  debugTimer = millis();
}

void loop() {
  // === UDP КОМАНДЫ ===
  handleCommandsUDP();
  
  // === UDP ЛИДАР ===
  handleLidarUDP();
  
  // === ОТПРАВКА БИНАРНЫХ ДАННЫХ ДАТЧИКОВ ===
  sendSensorData();
  
  // === СТАТИСТИКА ===
  printStats();
  
  yield();
}