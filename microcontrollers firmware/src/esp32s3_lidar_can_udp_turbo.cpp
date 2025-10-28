// ESP32-S3 LIDAR + CAN + UDP Bridge (TURBO)
// - RPLidar C1 UART 460800 → TCP server :3334 (максимальная оптимизация/IRAM/без блокировок)
// - Команды UDP :3333 → CAN (MCP2515) → Arduino
// - Сенсоры (энкодеры+IMU) UDP → хост :3335 (бинарный пакет)
// Платы: ESP32-S3. Требуются библиотеки: Adafruit_BNO055, Adafruit_Sensor, mcp2515

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <HardwareSerial.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ======================= CONFIG =======================
const char* FIRMWARE_VERSION = "v3.0-TURBO-UNIFIED";

// WiFi
const char* ssid     = "s548-poligon"; // поменяйте при необходимости
const char* password = "Bwog4581";     // поменяйте при необходимости

// Ports
static const uint16_t CMD_UDP_PORT     = 3333; // команды UDP
static const uint16_t SENSOR_UDP_PORT  = 3335; // сенсоры UDP
static const uint16_t LIDAR_TCP_PORT   = 3334; // лидар TCP

// LIDAR UART pins/speed
#define LIDAR_BAUDRATE 460800
#define LIDAR_RX_PIN 11
#define LIDAR_TX_PIN 12
HardwareSerial LidarSerial(1);

// CAN (MCP2515 on FSPI)
SPIClass mySpi = SPIClass(FSPI); // FSPI S3
MCP2515 mcp2515(7);              // CS=GPIO7
// FSPI pins: SCK=GPIO5, MISO=GPIO6, MOSI=GPIO4, CS=GPIO7 (как в вашем коде)
static const int FSPI_MOSI = 4;
static const int FSPI_MISO = 6;
static const int FSPI_SCLK = 5;
static const int FSPI_CS   = 7;

// IMU (BNO055 I2C on custom pins)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);
static const int I2C_SDA = 16;
static const int I2C_SCL = 15;

// ======================= NETWORK ======================
WiFiServer lidarServer(LIDAR_TCP_PORT);
WiFiClient lidarClient;
bool lidarClientConnected = false;

WiFiUDP udpCmd;    // приём команд (UDP:3333)
WiFiUDP udpSensor; // отправка сенсоров (UDP:3335)

IPAddress sensorDestIP; // куда слать UDP сенсоры
bool sensorDestKnown = false;

// ==================== LIDAR TURBO =====================
// Большие буферы, IRAM, минимальные задержки
static uint8_t streamBuffer[16384] __attribute__((aligned(4)));
static volatile size_t bufferIndex = 0;
static uint8_t tempPacket[5] __attribute__((aligned(4)));
static volatile int tempPacketIndex = 0;

static unsigned long lastSendTime = 0;
static const unsigned long MIN_SEND_INTERVAL_MS = 2;
static const size_t OPTIMAL_PACKET_SIZE = 1460;

static int packetCounter = 0;
static const int PACKET_THIN_FACTOR = 1;

static volatile unsigned long totalBytesSent = 0;
static volatile unsigned long totalPacketsSent = 0;
static unsigned long lastStatsTime = 0;
static const unsigned long STATS_INTERVAL_MS = 15000;

// RPLidar commands
static uint8_t STOP_CMD[]        = {0xA5, 0x25};
static uint8_t RESET_CMD[]       = {0xA5, 0x40};
static uint8_t START_SCAN_CMD[]  = {0xA5, 0x20};

// ==================== SENSORS/UDP =====================
struct __attribute__((packed)) SensorPacket {
  uint32_t timestamp_ms;  // L
  int32_t  left_abs;      // l (signed)
  int32_t  right_abs;     // l (signed)
  int16_t  yaw_deg;       // h (signed!) -180..+180
  int16_t  accel_mmps2;   // h (signed!) мм/с²
  uint8_t  packet_id;     // B
};

static uint8_t sensorPacketId = 0;
static const unsigned long SENSOR_SEND_INTERVAL_MS = 20; // 50 Гц
static unsigned long lastSensorSend = 0;
static volatile int32_t lastLeftAbs = 0;
static volatile int32_t lastRightAbs = 0;

// ====================== UTILS =========================
int convertedX(int angle) { // оставим при необходимости, но ниже используем 0..360
  if (angle <= 180) return angle;
  return angle - 360;
}

// ==================== LIDAR FAST PATH =================
IRAM_ATTR inline bool isValidRPLidarPacket() {
  uint8_t byte0 = tempPacket[0];
  uint8_t byte1 = tempPacket[1];
  return ((byte0 & 0x01) != ((byte0 >> 1) & 0x01)) && ((byte1 & 0x01) != 0);
}
IRAM_ATTR inline void fastMemCopy5(uint8_t* dest, const uint8_t* src) {
  *((uint32_t*)dest) = *((uint32_t*)src);
  dest[4] = src[4];
}
IRAM_ATTR void processCriticalLidarData() {
  const int MAX_BYTES_PER_CRITICAL_LOOP = 512;
  int bytesProcessed = 0;

  while (LidarSerial.available() && bytesProcessed < MAX_BYTES_PER_CRITICAL_LOOP) {
    uint8_t byte = LidarSerial.read();
    bytesProcessed++;
    tempPacket[tempPacketIndex++] = byte;

    if (tempPacketIndex >= 5) {
      if (isValidRPLidarPacket()) {
        packetCounter++;
        if (packetCounter % PACKET_THIN_FACTOR == 0) {
          if (bufferIndex <= sizeof(streamBuffer) - 5) {
            fastMemCopy5(&streamBuffer[bufferIndex], tempPacket);
            bufferIndex += 5;
          }
        }
        tempPacketIndex = 0;
      } else {
        memmove(tempPacket, tempPacket + 1, 4);
        tempPacketIndex = 4;
      }
    }
  }
}

// ===================== LIDAR INIT =====================
void initializeLidar() {
  Serial.println("[LIDAR] Stop...");
  LidarSerial.write(STOP_CMD, 2);
  delay(100);

  while (LidarSerial.available()) LidarSerial.read();

  Serial.println("[LIDAR] Reset...");
  LidarSerial.write(RESET_CMD, 2);
  delay(1000);

  while (LidarSerial.available()) {
    uint8_t b = LidarSerial.read();
    Serial.printf("%02X ", b);
  }
  Serial.println();

  Serial.println("[LIDAR] Start scan...");
  LidarSerial.write(START_SCAN_CMD, 2);
  delay(100);

  // quick probe
  unsigned long start = millis();
  int cnt = 0;
  while (millis() - start < 500) {
    if (LidarSerial.available()) {
      LidarSerial.read();
      cnt++;
    }
    delay(1);
  }
  Serial.printf("[LIDAR] Probe bytes: %d\n", cnt);
}

// ===================== CAN / IMU ======================
void sendCommandToArduino(float linear_x, float angular_z) {
  struct can_frame cmdFrame;
  cmdFrame.can_id = 0x200;
  cmdFrame.can_dlc = 8;
  memcpy(&cmdFrame.data[0], &linear_x, 4);
  memcpy(&cmdFrame.data[4], &angular_z, 4);
  mcp2515.sendMessage(&cmdFrame);
  Serial.println("sent com to ard");
}
void sendSensorPacketIfDue() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  int yaw = convertedX((int)euler.x());  // -180..+180
  int ax_mmps2 = (int)(accel.y() * 1000.0);  // м/с² → мм/с²

  SensorPacket pkt;
  pkt.timestamp_ms = millis();
  pkt.left_abs     = lastLeftAbs;
  pkt.right_abs    = lastRightAbs;
  pkt.yaw_deg      = (int16_t)yaw;        // ✅ ЯВНОЕ приведение
  pkt.accel_mmps2  = (int16_t)ax_mmps2;   // ✅ ЯВНОЕ приведение
  pkt.packet_id    = sensorPacketId++;

  udpSensor.beginPacket(sensorDestIP, SENSOR_UDP_PORT);
  udpSensor.write((const uint8_t*)&pkt, sizeof(pkt));
  udpSensor.endPacket();
  
  Serial.printf("✅ UDP: L=%d R=%d yaw=%d° acc=%d mm/s²\n", 
                lastLeftAbs, lastRightAbs, yaw, ax_mmps2);
}
void pollCAN() {
  struct can_frame canMsg;
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x100 && canMsg.can_dlc == 8) {
      Serial.println("got enc");
      int32_t left, right;
      memcpy(&left, &canMsg.data[0], 4);
      memcpy(&right, &canMsg.data[4], 4);
      lastLeftAbs = left;
      lastRightAbs = right;
  sendSensorPacketIfDue();
    }
  }
}


// ================ UDP COMMANDS (3333) =================
void handleUdpCommands() {
  int packetSize = udpCmd.parsePacket();
  if (!packetSize) return;

  // запомним IP источника → туда будем слать сенсоры
  sensorDestIP = udpCmd.remoteIP();
  sensorDestKnown = true;

  static char buf[128];
  int len = udpCmd.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = 0;

  // ожидается "linear_x angular_z"
  char* p = buf;
  while (*p == ' ' || *p == '\t') p++;
  char* space = strchr(p, ' ');
  if (space) {
    *space = 0;
    float lin = atof(p);
    float ang = atof(space + 1);
    Serial.println("com is right, sending to ard");
    sendCommandToArduino(lin, ang);
  }
}

// ======================= SETUP ========================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("=== ESP32-S3 LIDAR+CAN UDP TURBO ===");
  Serial.printf("Version: %s\n", FIRMWARE_VERSION);

  // CPU / Watchdog / WiFi perf
  setCpuFrequencyMhz(240);
  esp_task_wdt_deinit();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);

  // WiFi connect
  Serial.printf("WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) {
    delay(200);
    Serial.print(".");
  }
  Serial.printf("\nWiFi: %s  IP: %s  RSSI: %d\n",
                WiFi.status() == WL_CONNECTED ? "OK" : "FAIL",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Restarting...");
    ESP.restart();
  }

  // UDP
  udpCmd.begin(CMD_UDP_PORT);
  // udpSensor не нужно bind — только отправка

  // TCP LIDAR
  lidarServer.begin();
  Serial.printf("TCP LIDAR server ready :%d\n", LIDAR_TCP_PORT);

  // UART LIDAR
    LidarSerial.setRxBufferSize(8192);

  LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  LidarSerial.setTimeout(1);
  uart_set_rx_full_threshold(UART_NUM_1, 120);
  uart_set_rx_timeout(UART_NUM_1, 2);

  // CAN
  mySpi.begin(FSPI_MOSI, FSPI_MISO, FSPI_SCLK, FSPI_CS);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  // IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  if (bno.begin()) {
    bno.setMode(OPERATION_MODE_NDOF);
    Serial.println("IMU: OK");
  } else {
    Serial.println("IMU: NOT FOUND");
  }

  initializeLidar();

  lastStatsTime = millis();
  Serial.println("=== TURBO MODE ACTIVE ===");
}

// ======================== LOOP ========================
void loop() {
  unsigned long now = millis();

  // LIDAR TCP accept/disconnect
  if (!lidarClientConnected) {
    WiFiClient c = lidarServer.available();
    if (c) {
      lidarClient = c;
      lidarClientConnected = true;
      lidarClient.setNoDelay(true);
      lastSendTime = now;
      bufferIndex = 0;
      tempPacketIndex = 0;
      Serial.printf("[TCP] LIDAR client: %s\n", lidarClient.remoteIP().toString().c_str());

      // Автоматически используем этот IP для UDP сенсоров, если ещё не известен
      if (!sensorDestKnown) {
        sensorDestIP = lidarClient.remoteIP();
        sensorDestKnown = true;
      }
    }
  } else if (!lidarClient.connected()) {
    lidarClient.stop();
    lidarClientConnected = false;
    Serial.println("[TCP] LIDAR disconnected");
  }

  // Fast LIDAR path
  if (lidarClientConnected) {
    processCriticalLidarData();

    bool shouldSend = (bufferIndex >= OPTIMAL_PACKET_SIZE) ||
                      (bufferIndex > 0 && (now - lastSendTime) >= MIN_SEND_INTERVAL_MS);
    if (shouldSend) {
      size_t sent = lidarClient.write(streamBuffer, bufferIndex);
      if (sent == bufferIndex) {
        totalBytesSent += bufferIndex;
        totalPacketsSent++;
      } else if (sent == 0) {
        lidarClientConnected = false;
      }
      bufferIndex = 0;
      lastSendTime = now;
    }

    // Команды для лидара от клиента (прозрачный мост)
    while (lidarClient.available()) {
      uint8_t b = lidarClient.read();
      LidarSerial.write(b);
    }
  } else {
    while (LidarSerial.available()) LidarSerial.read();
    bufferIndex = 0;
    tempPacketIndex = 0;
    packetCounter = 0;
  }

  // UDP команды → CAN
  handleUdpCommands();

  // CAN опрос
  pollCAN();

  // Отправка сенсоров UDP (50 Гц)

  // Статистика
  if (now - lastStatsTime >= STATS_INTERVAL_MS) {
    Serial.printf("[TURBO] %lu KB/s, %lu pkt/s, heap: %d KB, UDPdest:%s\n",
                  (totalBytesSent / 1024) * 1000 / STATS_INTERVAL_MS,
                  totalPacketsSent * 1000 / STATS_INTERVAL_MS,
                  ESP.getFreeHeap() / 1024,
                  sensorDestKnown ? sensorDestIP.toString().c_str() : "-");
    totalBytesSent = 0;
    totalPacketsSent = 0;
    lastStatsTime = now;
  }

  // никаких delay() — максимум производительности
}