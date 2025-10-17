// ИСПРАВЛЕННАЯ версия с бинарными пакетами датчиков
// Основные изменения:
// 1. Добавлена бинарная структура SensorDataPacket
// 2. Заменена отправка датчиков на бинарный формат
// 3. Добавлены ID пакетов для отслеживания

// === ДОБАВИТЬ В НАЧАЛО ФАЙЛА ===
// Бинарная структура для датчиков (соответствует Python формату '<LLLHHB')
struct SensorDataPacket {
    uint32_t timestamp;     // millis()
    uint32_t left_ticks;    // Левый энкодер
    uint32_t right_ticks;   // Правый энкодер  
    uint16_t yaw_raw;       // yaw * 10 (для точности)
    uint16_t accel_raw;     // accel * 1000 (для точности)
    uint8_t packet_id;      // Счетчик пакетов
};

static uint8_t sensor_packet_id = 0;  // Счетчик пакетов

// === ЗАМЕНИТЬ ФУНКЦИЮ sendSensorData() ===
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
      
      // === НОВОЕ: Формируем БИНАРНЫЙ пакет ===
      SensorDataPacket packet;
      packet.timestamp = millis();
      packet.left_ticks = (uint32_t)left;
      packet.right_ticks = (uint32_t)right;
      packet.yaw_raw = (uint16_t)((yaw + 360) * 10) % 3600;  // 0-3599 (0.1 град точность)
      packet.accel_raw = (uint16_t)((x_acc + 10.0) * 1000);  // -10 to +10 m/s² с точностью 0.001
      packet.packet_id = sensor_packet_id++;
      
      // Отправляем бинарный пакет
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

// === ДОПОЛНИТЕЛЬНЫЕ УЛУЧШЕНИЯ ===

// В функции setup() добавить проверку размера структуры:
void setup() {
  // ... существующий код ...
  
  // Проверяем размер структуры (должно быть 17 байт)
  Serial.printf("📦 SensorDataPacket size: %d bytes (expected: 17)\n", sizeof(SensorDataPacket));
  if (sizeof(SensorDataPacket) != 17) {
    Serial.println("❌ ОШИБКА: Неправильный размер структуры датчиков!");
    Serial.println("   Проверьте выравнивание структуры в компиляторе");
  }
  
  // ... остальной код setup() ...
}

// Улучшенная статистика с информацией о бинарных пакетах
void printStats() {
  if (millis() - debugTimer < 10000) return;
  
  Serial.println("\n╔═══════ UDP STATUS ═══════╗");
  Serial.printf("║ WiFi: %s (%ddBm) ║\n", 
                WiFi.status() == WL_CONNECTED ? "✅" : "❌",
                WiFi.RSSI());
  
  if (has_cmd_client) {
    Serial.printf("║ ROS2: %s ║\n", last_cmd_ip.toString().c_str());
  } else {
    Serial.println("║ ROS2: 🔍 Ожидание клиента ║");
  }
  
  Serial.println("╠═════════════════════════╣");
  
  static unsigned long lastRx = 0;
  unsigned long rxPerSec = (lidarBytesRx - lastRx) / 10;  // За 10 секунд
  lastRx = lidarBytesRx;
  
  Serial.printf("║ LIDAR RX: %6lu B/s ║\n", rxPerSec);
  Serial.printf("║ LIDAR TX: %6lu B   ║\n", lidarBytesTx);
  Serial.printf("║ SENSOR: #%-6d    ║\n", sensor_packet_id);  // Счетчик пакетов датчиков
  
  if (lastDataTime > 0) {
    unsigned long ago = millis() - lastDataTime;
    if (ago < 1000) {
      Serial.println("║ Status: ✅ ACTIVE     ║");
    } else {
      Serial.printf("║ Status: ⚠️ %4lus ago ║\n", ago/1000);
    }
  }
  
  Serial.println("╚═════════════════════════╝");
  debugTimer = millis();
}