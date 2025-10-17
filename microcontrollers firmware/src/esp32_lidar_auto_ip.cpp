// ESP32 LIDAR Bridge с автоопределением IP (как в эхо-сервере)
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

// === БИНАРНАЯ СТРУКТУРА ДАТЧИКОВ ===
struct SensorDataPacket {
    uint32_t timestamp;     // millis()
    uint32_t left_ticks;    // Левый энкодер
    uint32_t right_ticks;   // Правый энкодер  
    uint16_t yaw_raw;       // yaw * 10 (0.1 град точность)
    uint16_t accel_raw;     // (accel + 10) * 1000 (0.001 м/с² точность)
    uint8_t packet_id;      // Счетчик пакетов 0-255
} __attribute__((packed));  // = 17 байт точно!

static uint8_t sensor_packet_id = 0;

// UDP объекты
WiFiUDP cmdUdp;
WiFiUDP lidarUdp;
WiFiUDP sensorUdp;

// UDP порты
const uint16_t CMD_PORT = 3333;       // Порт для приема команд
const uint16_t LIDAR_PORT = 3334;     // Порт для передачи LIDAR данных
const uint16_t SENSOR_PORT = 3335;    // Порт для передачи данных датчиков

// Динамический IP клиента (автоопределение как в эхо-сервере)
IPAddress client_ip;          // IP определяется из первой команды
bool has_client = false;      // Есть ли активный клиент

// === RPLIDAR НА ПРОГРАММНОМ UART ===
#define LIDAR_RX_PIN 12  // Желтый (TX лидара)
#define LIDAR_TX_PIN 11  // Зеленый (RX лидара)
#define LIDAR_BAUDRATE 460800  

HardwareSerial LidarSerial(1); 

SPIClass mySpi = SPIClass(FSPI);
MCP2515 mcp2515(7);
struct can_frame canMsg;

// UDP буферы
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

void testLidar() {
  Serial.println("\n╔═══════════════════════════════╗");
  Serial.println("║  RPLIDAR C1 CONNECTION TEST   ║");
  Serial.println("╚═══════════════════════════════╝");
  Serial.printf("HardwareSerial: RX=%d TX=%d %d baud\n", 
                LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUDRATE);
  
  delay(500);
  while (LidarSerial.available()) LidarSerial.read();
  
  // 1. STOP
  Serial.print("\n1️⃣ STOP... ");
  LidarSerial.write(0xA5); LidarSerial.write(0x25); LidarSerial.flush();
  delay(200);
  int bytes = 0;
  while (LidarSerial.available()) { LidarSerial.read(); bytes++; }
  Serial.printf("(%d bytes)\n", bytes);
  
  // 2. START SCAN
  Serial.print("2️⃣ START SCAN... ");
  LidarSerial.write(0xA5); LidarSerial.write(0x20); LidarSerial.flush();
  delay(500);
  
  bytes = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) {
    if (LidarSerial.available()) { LidarSerial.read(); bytes++; }
  }
  
  int bytesPerSec = bytes / 2;
  Serial.printf("%d B/s\n", bytesPerSec);
  
  if (bytesPerSec > 1000) {
    Serial.println("✅ EXCELLENT! Full speed!");
  } else if (bytesPerSec > 200) {
    Serial.println("⚠️ Working but slow");
  } else {
    Serial.println("❌ Low/no data");
  }
  
  Serial.println("═══════════════════════════════\n");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n╔════════════════════════════════════╗");
  Serial.println("║  ESP32 LIDAR Bridge Auto IP       ║");
  Serial.println("║     Binary Sensors + Auto IP      ║");
  Serial.println("╚════════════════════════════════════╝");
  
  // Проверяем размер структуры датчиков
  Serial.printf("\n🔧 SensorDataPacket size: %d bytes (expected: 17)\n", sizeof(SensorDataPacket));
  if (sizeof(SensorDataPacket) != 17) {
    Serial.println("❌ КРИТИЧЕСКАЯ ОШИБКА: Неправильный размер структуры!");
    while(1) delay(1000);
  } else {
    Serial.println("✅ Размер структуры корректен (17 байт)");
  }
  
  // === HARDWARE SERIAL для LIDAR ===
  Serial.println("\n🔧 Init LIDAR HardwareSerial...");
  LidarSerial.begin(460800, SERIAL_8N1, 11, 12); 
  Serial.printf("✅ LIDAR: RX=%d TX=%d @ %d baud\n", 
                LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUDRATE);
  
  // === ТЕСТ LIDAR ===
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
    Serial.printf("\n✅ ESP32 IP: %s\n", WiFi.localIP().toString().c_str());
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
  Serial.println("   Client IP: Auto-detect from first command");
  
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
  Serial.println("║ Send any UDP command to register   ║");
  Serial.println("╚════════════════════════════════════╝");
  
  // === Автоматический запуск LIDAR ===
  Serial.println("⚙️ Автозапуск LIDAR...");
  delay(1000);
  LidarSerial.write(0xA5); LidarSerial.write(0x25); delay(100);  // STOP
  LidarSerial.write(0xA5); LidarSerial.write(0x20);             // START SCAN
  Serial.println("✅ LIDAR автозапуск выполнен");
  
  debugTimer = millis();
}

// === UDP LIDAR МОСТ (как в оригинале) ===
void handleLidarUDP() {
  static int lidarBufferIndex = 0;
  static unsigned long lastSendTime = 0;
  const unsigned long SEND_INTERVAL = 10; // Отправлять каждые 10мс
  
  // Читаем данные от LIDAR
  int readBytes = 0;
  unsigned long readStart = millis();
  
  while (LidarSerial.available() && (millis() - readStart < 3) && lidarBufferIndex < LIDAR_BUFFER_SIZE) {
    uint8_t byte = LidarSerial.read();
    lidarBuffer[lidarBufferIndex++] = byte;
    lidarBytesRx++;
    readBytes++;
    
    if (firstData) {
      Serial.println("🎉 FIRST LIDAR DATA!");
      firstData = false;
    }
    
    if (lidarBufferIndex >= LIDAR_BUFFER_SIZE - 50) {
      break;
    }
  }
  
  // Отправляем пакеты когда есть данные и клиент зарегистрирован
  bool shouldSend = false;
  
  if (lidarBufferIndex > 0 && has_client) {
    if (lidarBufferIndex >= 200 || (millis() - lastSendTime >= SEND_INTERVAL)) {
      shouldSend = true;
    }
  }
  
  if (shouldSend) {
    // Ограничиваем размер пакета для надежности
    int sendSize = min(lidarBufferIndex, 400);  // Макс 400 байт за раз
    
    int result = lidarUdp.beginPacket(client_ip, LIDAR_PORT);
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
        Serial.println("⚠️ LIDAR UDP send failed");
        lidarBufferIndex = 0;
      }
    } else {
      Serial.println("⚠️ LIDAR UDP begin failed");
      lidarBufferIndex = 0;
    }
    
    lastSendTime = millis();
    lastDataTime = millis();
  }
}

// === UDP КОМАНДЫ (С АВТООПРЕДЕЛЕНИЕМ IP) ===
void handleCommandsUDP() {
  int packetSize = cmdUdp.parsePacket();
  if (packetSize > 0) {
    // АВТООПРЕДЕЛЕНИЕ IP КЛИЕНТА (как в эхо-сервере)
    IPAddress sender_ip = cmdUdp.remoteIP();
    if (!has_client || client_ip != sender_ip) {
      client_ip = sender_ip;
      has_client = true;
      Serial.printf("🔗 Client registered: %s (auto-detected)\n", client_ip.toString().c_str());
    }
    
    int len = cmdUdp.read(cmdBuffer, CMD_BUFFER_SIZE - 1);
    
    if (len > 0) {
      // Проверяем тип команды
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
            Serial.printf("→Arduino: %.3f %.3f\n", lin, ang);
          }
        }
      } else {
        // Бинарные команды для LIDAR
        for (int i = 0; i < len; i++) {
          LidarSerial.write((uint8_t)cmdBuffer[i]);
        }
        Serial.printf("→LIDAR: %d bytes\n", len);
      }
    }
  }
}

// === ОТПРАВКА БИНАРНЫХ ДАННЫХ ДАТЧИКОВ ===
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
  
  if (has_client) {
    Serial.printf("║ Client: %s ║\n", client_ip.toString().c_str());
  } else {
    Serial.println("║ Client: 🔍 Waiting...          ║");
  }
  
  Serial.println("╠════════════════════════════════╣");
  
  static unsigned long lastRx = 0;
  unsigned long rxPerSec = (lidarBytesRx - lastRx) / 10;
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
  // === UDP КОМАНДЫ (с автоопределением IP) ===
  handleCommandsUDP();
  
  // === UDP ЛИДАР (отправка только зарегистрированному клиенту) ===
  handleLidarUDP();
  
  // === ОТПРАВКА БИНАРНЫХ ДАННЫХ ДАТЧИКОВ ===
  sendSensorData();
  
  // === СТАТИСТИКА ===
  printStats();
  
  yield();
}