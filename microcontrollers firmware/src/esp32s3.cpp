// esp32s3_rplidar_bridge_final.cpp - ПРОЗРАЧНЫЙ МОСТ ДЛЯ RPLIDAR C1
#include <WiFi.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>

const char* ssid = "s548-poligon";
const char* password = "Bwog4581";

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

// WiFi серверы
WiFiServer cmdServer(3333);
WiFiServer lidarServer(3334);
WiFiClient cmdClient;
WiFiClient lidarClient;

// === RPLIDAR НА ПРОГРАММНОМ UART ===
#define LIDAR_RX_PIN 12  // Желтый (TX лидара)
#define LIDAR_TX_PIN 11  // Зеленый (RX лидара)
#define LIDAR_BAUDRATE 460800  // RPLidar C1 стандартная скорость

HardwareSerial LidarSerial(1); 

SPIClass mySpi = SPIClass(FSPI);
MCP2515 mcp2515(7);
struct can_frame canMsg;

String wifiBuffer = "";

// Буферы
#define LIDAR_BUFFER_SIZE 512
uint8_t lidarTxBuffer[LIDAR_BUFFER_SIZE];
int lidarTxIndex = 0;
unsigned long lastLidarFlush = 0;

// Статистика
unsigned long debugTimer = 0;
unsigned long lidarBytesRx = 0;
unsigned long lidarBytesTx = 0;
unsigned long lastDataTime = 0;
bool firstData = true;

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
  Serial.printf("SoftSerial: RX=%d TX=%d %d baud\n", 
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
  Serial.println("║      SoftwareSerial Version       ║");
  Serial.println("╚════════════════════════════════════╝");
  
  // === SOFTWARE SERIAL ===
  Serial.println("\n🔧 Init SoftwareSerial...");
  LidarSerial.begin(460800, SERIAL_8N1, 11, 12); 
  
  Serial.printf("✅ SoftSerial: RX=%d TX=%d @ %d baud\n", 
                LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_BAUDRATE);
  Serial.println("⚠️  460800 baud on SoftSerial = possible data loss!");
  
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
  
  // === Серверы ===
  cmdServer.begin();
  lidarServer.begin();
  Serial.println("\n✅ Servers:");
  Serial.println("   :3333 - Commands");
  Serial.println("   :3334 - LIDAR");
  
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
  Serial.printf("║ READY! %s         ║\n", WiFi.localIP().toString().c_str());
  Serial.println("╚════════════════════════════════════╝\n");
  
  debugTimer = millis();
}

// === ПРОЗРАЧНЫЙ МОСТ ЛИДАРА ===
void handleLidarBridge() {
  static bool connected = false;
  
  // Подключение
  if (!lidarClient || !lidarClient.connected()) {
    if (connected) {
      Serial.println("🔴 Lidar client lost");
      connected = false;
      
      // STOP лидар
      LidarSerial.write(0xA5);
      LidarSerial.write(0x25);
    }
    
    WiFiClient newClient = lidarServer.available();
    if (newClient) {
      lidarClient = newClient;
      Serial.printf("\n🔴 LIDAR: %s\n", lidarClient.remoteIP().toString().c_str());
      connected = true;
      lidarTxIndex = 0;
      firstData = true;
    }
    return;
  }
  
  // ROS2 → LIDAR (команды)
  int cmdBytes = 0;
  while (lidarClient.available()) {
    uint8_t byte = lidarClient.read();
    LidarSerial.write(byte);
    lidarBytesTx++;
    cmdBytes++;
  }
  
  if (cmdBytes > 0) {
    Serial.printf("→LIDAR: %d bytes\n", cmdBytes);
  }
  
  // LIDAR → ROS2 (данные)
  // Читаем небольшими порциями (SoftSerial ограничение)
  int readBytes = 0;
  unsigned long readStart = millis();
  
  while (LidarSerial.available() && (millis() - readStart < 2)) {
    uint8_t byte = LidarSerial.read();
    lidarTxBuffer[lidarTxIndex++] = byte;
    lidarBytesRx++;
    readBytes++;
    
    if (firstData) {
      Serial.println("🎉 FIRST DATA!");
      firstData = false;
    }
    
    // Отправка при заполнении
    if (lidarTxIndex >= LIDAR_BUFFER_SIZE) {
      lidarClient.write(lidarTxBuffer, lidarTxIndex);
      lidarTxIndex = 0;
      lastLidarFlush = millis();
    }
    
    // Микропауза для стабильности
    if (readBytes % 32 == 0) {
      delayMicroseconds(50);
    }
  }
  
  if (readBytes > 0) {
    lastDataTime = millis();
  }
  
  // Периодическая отправка (каждые 2мс)
  if (lidarTxIndex > 0 && (millis() - lastLidarFlush) > 2) {
    lidarClient.write(lidarTxBuffer, lidarTxIndex);
    lidarTxIndex = 0;
    lastLidarFlush = millis();
  }
}

// === СТАТИСТИКА ===
void printStats() {
  if (millis() - debugTimer < 5000) return;
  
  Serial.println("\n╔═══════ STATUS ═══════╗");
  Serial.printf("║ WiFi: %s (%ddBm) ║\n", 
                WiFi.status() == WL_CONNECTED ? "✅" : "❌",
                WiFi.RSSI());
  Serial.printf("║ CMD:   %s         ║\n",
                cmdClient && cmdClient.connected() ? "✅" : "⭕");
  Serial.printf("║ LIDAR: %s         ║\n",
                lidarClient && lidarClient.connected() ? "✅" : "⭕");
  
  if (lidarClient && lidarClient.connected()) {
    Serial.println("╠══════════════════════╣");
    
    static unsigned long lastRx = 0;
    unsigned long rxPerSec = (lidarBytesRx - lastRx) / 5;
    lastRx = lidarBytesRx;
    
    Serial.printf("║ RX: %6lu B/s    ║\n", rxPerSec);
    Serial.printf("║ TX: %6lu bytes  ║\n", lidarBytesTx);
    
    if (lastDataTime > 0) {
      unsigned long ago = millis() - lastDataTime;
      if (ago < 500) {
        Serial.println("║ Flow: ✅ ACTIVE    ║");
      } else {
        Serial.printf("║ Flow: ⚠️ %4lus ago ║\n", ago/1000);
      }
    }
  }
  
  Serial.println("╚══════════════════════╝");
  debugTimer = millis();
}

void loop() {
  // === КОМАНДЫ ===
  if (!cmdClient || !cmdClient.connected()) {
    cmdClient = cmdServer.available();
    if (cmdClient) {
      Serial.printf("\n📡 CMD: %s\n", cmdClient.remoteIP().toString().c_str());
    }
  }
  
  if (cmdClient && cmdClient.connected() && cmdClient.available()) {
    while (cmdClient.available()) {
      char c = cmdClient.read();
      if (c == '\n') {
        wifiBuffer.trim();
        if (wifiBuffer.length() > 0) {
          int idx = wifiBuffer.indexOf(' ');
          if (idx > 0) {
            float lin = wifiBuffer.substring(0, idx).toFloat();
            float ang = wifiBuffer.substring(idx + 1).toFloat();
            sendCommandToArduino(lin, ang);
          }
        }
        wifiBuffer = "";
      } else if (c != '\r') {
        wifiBuffer += c;
      }
    }
  }
  
  // === CAN + IMU ===
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x100 && canMsg.can_dlc == 8) {
      int32_t left, right;
      memcpy(&left, &canMsg.data[0], 4);
      memcpy(&right, &canMsg.data[4], 4);
      
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      
      int yaw = convertedX((int)euler.x());
      float x_acc = accel.y();
      
      if (cmdClient && cmdClient.connected()) {
        char buf[80];
        snprintf(buf, sizeof(buf), "%ld %ld %d %.3f\n", left, right, yaw, x_acc);
        cmdClient.write((uint8_t*)buf, strlen(buf));
      }
    }
  }
  
  // === ЛИДАР ===
  handleLidarBridge();
  
  // === СТАТИСТИКА ===
  printStats();
  
  yield();
}