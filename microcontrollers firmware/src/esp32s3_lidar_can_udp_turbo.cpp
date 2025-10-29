// ESP32-S3 LIDAR + CAN + UDP Bridge (FIXED TURBO + CAN DIAGNOSTICS)
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
const char* FIRMWARE_VERSION = "v3.2-CAN-DIAG";

const char* ssid     = "s548-poligon";
const char* password = "Bwog4581";

static const uint16_t CMD_UDP_PORT     = 3333;
static const uint16_t SENSOR_UDP_PORT  = 3335;
static const uint16_t LIDAR_TCP_PORT   = 3334;

#define LIDAR_BAUDRATE 460800
#define LIDAR_RX_PIN 11
#define LIDAR_TX_PIN 12
HardwareSerial LidarSerial(1);

SPIClass mySpi = SPIClass(FSPI);
MCP2515 mcp2515(7);
static const int FSPI_MOSI = 4;
static const int FSPI_MISO = 6;
static const int FSPI_SCLK = 5;
static const int FSPI_CS   = 7;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);
static const int I2C_SDA = 16;
static const int I2C_SCL = 15;

// ======================= NETWORK ======================
WiFiServer lidarServer(LIDAR_TCP_PORT);
WiFiClient lidarClient;
bool lidarClientConnected = false;

WiFiUDP udpCmd;
WiFiUDP udpSensor;

IPAddress sensorDestIP;
bool sensorDestKnown = false;

// ==================== CAN DIAGNOSTICS =================
struct CANStats {
  unsigned long cmd_tx_success = 0;
  unsigned long cmd_tx_failed = 0;
  unsigned long enc_rx_success = 0;
  unsigned long enc_rx_failed = 0;
  unsigned long last_cmd_time = 0;
  unsigned long last_enc_time = 0;
  float last_linear = 0.0;
  float last_angular = 0.0;
};
CANStats can_stats;
bool can_initialized = false;

// ==================== LIDAR TURBO =====================
static uint8_t streamBuffer[32768] __attribute__((aligned(4)));
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
static const unsigned long STATS_INTERVAL_MS = 10000;  // 10 сек

static uint8_t STOP_CMD[]        = {0xA5, 0x25};
static uint8_t RESET_CMD[]       = {0xA5, 0x40};
static uint8_t START_SCAN_CMD[]  = {0xA5, 0x20};

// ==================== SENSORS/UDP =====================
struct __attribute__((packed)) SensorPacket {
  uint32_t timestamp_ms;
  int32_t  left_abs;
  int32_t  right_abs;
  int16_t  yaw_deg;
  int16_t  accel_mmps2;
  uint8_t  packet_id;
};

static uint8_t sensorPacketId = 0;
static const unsigned long SENSOR_SEND_INTERVAL_MS = 20;
static unsigned long lastSensorSend = 0;
static volatile int32_t lastLeftAbs = 0;
static volatile int32_t lastRightAbs = 0;

SemaphoreHandle_t encoderMutex;

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
  while (LidarSerial.available()) {
    uint8_t byte = LidarSerial.read();
    tempPacket[tempPacketIndex++] = byte;

    if (tempPacketIndex >= 5) {
      if (isValidRPLidarPacket()) {
        packetCounter++;
        if (packetCounter % PACKET_THIN_FACTOR == 0) {
          if (bufferIndex <= sizeof(streamBuffer) - 5) {
            fastMemCopy5(&streamBuffer[bufferIndex], tempPacket);
            bufferIndex += 5;
          } else {
            break;
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

  while (LidarSerial.available()) LidarSerial.read();

  Serial.println("[LIDAR] Start scan...");
  LidarSerial.write(START_SCAN_CMD, 2);
  delay(100);

  Serial.println("[LIDAR] Init complete");
}

// ===================== CAN INIT & TEST ================
bool initCAN() {
  Serial.println("\n[CAN] Initializing...");
  
  // ✅ ПРАВИЛЬНЫЙ ПОРЯДОК: begin(SCK, MISO, MOSI, SS)
  mySpi.begin(4, 6, 5, 7);
  delay(10);
  
  // Reset
  MCP2515::ERROR err = mcp2515.reset();
  if (err != MCP2515::ERROR_OK) {
    Serial.printf("❌ CAN reset FAILED: %d\n", err);
    return false;
  }
  Serial.println("✅ CAN reset OK");
  delay(10);
  
  // Set bitrate
  err = mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  if (err != MCP2515::ERROR_OK) {
    Serial.printf("❌ CAN setBitrate FAILED: %d\n", err);
    return false;
  }
  Serial.println("✅ CAN bitrate OK (125KBPS, 8MHz)");
  delay(10);
  
  // Set normal mode
  err = mcp2515.setNormalMode();
  if (err != MCP2515::ERROR_OK) {
    Serial.printf("❌ CAN setNormalMode FAILED: %d\n", err);
    return false;
  }
  Serial.println("✅ CAN mode OK (NORMAL)");
  

  
  return true;
}

// Тест отправки CAN
void testCANSend() {
  Serial.println("\n[CAN TEST] Sending test frame...");
  
  struct can_frame testFrame;
  testFrame.can_id = 0x200;
  testFrame.can_dlc = 8;
  
  float test_lin = 0.0;
  float test_ang = 0.0;
  memcpy(&testFrame.data[0], &test_lin, 4);
  memcpy(&testFrame.data[4], &test_ang, 4);
  
  MCP2515::ERROR result = mcp2515.sendMessage(&testFrame);
  
  if (result == MCP2515::ERROR_OK) {
    Serial.println("✅ CAN test send OK");
  } else {
    Serial.printf("❌ CAN test send FAILED: %d\n", result);
  }
}

// ===================== CAN / IMU ======================
void sendCommandToArduino(float linear_x, float angular_z) {
  if (!can_initialized) {
    Serial.println("⚠️ CAN not initialized!");
    can_stats.cmd_tx_failed++;
    return;
  }
  
  struct can_frame cmdFrame;
  cmdFrame.can_id = 0x200;  // ID как в Arduino
  cmdFrame.can_dlc = 8;
  memcpy(&cmdFrame.data[0], &linear_x, 4);
  memcpy(&cmdFrame.data[4], &angular_z, 4);
  
  MCP2515::ERROR result = mcp2515.sendMessage(&cmdFrame);
  
  if (result == MCP2515::ERROR_OK) {
    can_stats.cmd_tx_success++;
    can_stats.last_cmd_time = millis();
    can_stats.last_linear = linear_x;
    can_stats.last_angular = angular_z;
    
    Serial.printf("✅ CAN TX: lin=%.3f ang=%.3f (total: %lu)\n", 
                  linear_x, angular_z, can_stats.cmd_tx_success);
  } else {
    can_stats.cmd_tx_failed++;
    Serial.printf("❌ CAN TX FAILED: %d (fails: %lu)\n", 
                  result, can_stats.cmd_tx_failed);
  }
}

void sendSensorPacketIfDue() {
  unsigned long now = millis();
  if (now - lastSensorSend < SENSOR_SEND_INTERVAL_MS) {
    return;
  }
  lastSensorSend = now;

  if (!sensorDestKnown) {
    return;
  }

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  int yaw = (int)euler.x();
  if (yaw > 180) yaw -= 360;
  int ax_mmps2 = (int)(accel.y() * 1000.0);

  SensorPacket pkt;
  pkt.timestamp_ms = now;
  
  if (xSemaphoreTake(encoderMutex, 5 / portTICK_PERIOD_MS) == pdTRUE) {
    pkt.left_abs  = lastLeftAbs;
    pkt.right_abs = lastRightAbs;
    xSemaphoreGive(encoderMutex);
  } else {
    pkt.left_abs  = 0;
    pkt.right_abs = 0;
  }
  
  pkt.yaw_deg      = (int16_t)yaw;
  pkt.accel_mmps2  = (int16_t)ax_mmps2;
  pkt.packet_id    = sensorPacketId++;

  udpSensor.beginPacket(sensorDestIP, SENSOR_UDP_PORT);
  udpSensor.write((const uint8_t*)&pkt, sizeof(pkt));
  udpSensor.endPacket();
}

void pollCAN() {
  if (!can_initialized) {
    return;
  }
  
  struct can_frame canMsg;
  MCP2515::ERROR result = mcp2515.readMessage(&canMsg);
  
  if (result == MCP2515::ERROR_OK) {
    // Получили сообщение
    if (canMsg.can_id == 0x100 && canMsg.can_dlc == 8) {
      // Энкодеры от Arduino
      int32_t left, right;
      memcpy(&left, &canMsg.data[0], 4);
      memcpy(&right, &canMsg.data[4], 4);
      
      if (xSemaphoreTake(encoderMutex, 5 / portTICK_PERIOD_MS) == pdTRUE) {
        lastLeftAbs = left;
        lastRightAbs = right;
        xSemaphoreGive(encoderMutex);
      }
      
      can_stats.enc_rx_success++;
      can_stats.last_enc_time = millis();
      
      Serial.printf("📥 CAN RX: L=%d R=%d (total: %lu)\n", 
                    left, right, can_stats.enc_rx_success);
    }
  } else if (result != MCP2515::ERROR_NOMSG) {
    // Ошибка чтения (не "нет сообщений")
    can_stats.enc_rx_failed++;
  }
}

// ================ UDP COMMANDS (3333) =================
void handleUdpCommands() {
  int packetSize = udpCmd.parsePacket();
  if (!packetSize) return;

  sensorDestIP = udpCmd.remoteIP();
  sensorDestKnown = true;

  static char buf[128];
  int len = udpCmd.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = 0;

  char* p = buf;
  while (*p == ' ' || *p == '\t') p++;
  char* space = strchr(p, ' ');
  if (space) {
    *space = 0;
    float lin = atof(p);
    float ang = atof(space + 1);
    
    Serial.printf("📡 UDP CMD: lin=%.3f ang=%.3f\n", lin, ang);
    sendCommandToArduino(lin, ang);
  }
}

// ✅ ЗАДАЧА ЛИДАРА
void lidarTask(void* parameter) {
  Serial.println("[TASK] LIDAR task started (Core 1, Priority 10)");
  
  for(;;) {
    unsigned long now = millis();

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

    if (lidarClientConnected) {
      processCriticalLidarData();

      bool shouldSend = (bufferIndex >= OPTIMAL_PACKET_SIZE) ||
                        (bufferIndex > 0 && (now - lastSendTime) >= MIN_SEND_INTERVAL_MS) ||
                        (bufferIndex > sizeof(streamBuffer) - 500);

      if (shouldSend && bufferIndex > 0) {
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

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ======================= SETUP ========================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n=== ESP32-S3 LIDAR+CAN UDP TURBO ===");
  Serial.printf("Version: %s\n", FIRMWARE_VERSION);

  setCpuFrequencyMhz(240);
  esp_task_wdt_deinit();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);

  Serial.printf("WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) {
    delay(200);
    Serial.print(".");
  }
  Serial.printf("\nWiFi: %s  IP: %s  RSSI: %d\n",
                WiFi.status() == WL_CONNECTED ? "OK" : "FAIL",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  udpCmd.begin(CMD_UDP_PORT);

  lidarServer.begin();
  Serial.printf("TCP LIDAR server ready :%d\n", LIDAR_TCP_PORT);

  LidarSerial.setRxBufferSize(16384);
  LidarSerial.begin(460800, SERIAL_8N1, 11, 12);
  LidarSerial.setTimeout(1);
  uart_set_rx_full_threshold(UART_NUM_1, 120);
  uart_set_rx_timeout(UART_NUM_1, 1);

  // ✅ ИНИЦИАЛИЗАЦИЯ CAN
  can_initialized = initCAN();
  if (can_initialized) {
    delay(500);
    testCANSend();  // Тестовая отправка
  } else {
    Serial.println("❌ CAN INITIALIZATION FAILED!");
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  if (bno.begin()) {
    bno.setMode(OPERATION_MODE_NDOF);
    Serial.println("IMU: OK");
  } else {
    Serial.println("IMU: NOT FOUND");
  }

  encoderMutex = xSemaphoreCreateMutex();

  initializeLidar();

  xTaskCreatePinnedToCore(
    lidarTask,
    "LidarTask",
    8192,
    NULL,
    10,
    NULL,
    1
  );

  lastStatsTime = millis();
  Serial.println("=== SYSTEM READY ===\n");
}

// ======================== LOOP ========================
void loop() {
  unsigned long now = millis();

  handleUdpCommands();
  pollCAN();
  sendSensorPacketIfDue();

  // ✅ РАСШИРЕННАЯ СТАТИСТИКА
  if (now - lastStatsTime >= STATS_INTERVAL_MS) {
    Serial.println("\n========== STATISTICS ==========");
    
    // LIDAR
    Serial.printf("[LIDAR] %lu KB/s, %lu pkt/s\n",
                  (totalBytesSent / 1024) * 1000 / STATS_INTERVAL_MS,
                  totalPacketsSent * 1000 / STATS_INTERVAL_MS);
    
    // CAN
    Serial.printf("[CAN] TX: %lu OK / %lu FAIL\n", 
                  can_stats.cmd_tx_success, can_stats.cmd_tx_failed);
    Serial.printf("[CAN] RX: %lu OK / %lu FAIL\n", 
                  can_stats.enc_rx_success, can_stats.enc_rx_failed);
    Serial.printf("[CAN] Last CMD: %.3f, %.3f (%lu ms ago)\n",
                  can_stats.last_linear, can_stats.last_angular,
                  now - can_stats.last_cmd_time);
    Serial.printf("[CAN] Last ENC: %lu ms ago\n", now - can_stats.last_enc_time);
    
    // SYSTEM
    Serial.printf("[SYS] Heap: %d KB, UDP dest: %s\n",
                  ESP.getFreeHeap() / 1024,
                  sensorDestKnown ? sensorDestIP.toString().c_str() : "-");
    Serial.println("================================\n");
    
    totalBytesSent = 0;
    totalPacketsSent = 0;
    lastStatsTime = now;
  }

  delay(1);
}