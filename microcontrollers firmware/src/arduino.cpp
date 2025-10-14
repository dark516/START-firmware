#include "motor_regulator.h"
#include "ros2_communication.hpp"
#include <SPI.h>
#include <mcp2515.h>

// ===== CAN =====
MCP2515 mcp2515(10);  // CS = D10
struct can_frame canMsg;

// ===== Моторы =====
Motor left_motor(7, 6);
Motor right_motor(8, 9);

// ===== Энкодеры =====
class Regulator;
extern Regulator left_regulator;
extern Regulator right_regulator;

Encoder left_enc(2, 4, [] { left_regulator.encoder.encoder_int(); }, false);
Encoder right_enc(3, 5, [] { right_regulator.encoder.encoder_int(); }, false);

// ===== PID =====
PID left_pid(4.0, 0.2, 0.04, 100);
PID right_pid(4.0, 0.2, 0.04, 100);

// ===== Регуляторы =====
Regulator left_regulator(left_motor, left_enc, left_pid);
Regulator right_regulator(right_motor, right_enc, right_pid);

// ===== Переменные скорости =====
float v_lin = 0.05;   // линейная скорость
float v_ang = 0.0;    // угловая скорость

// ===== Для проверки изменений =====
int32_t last_left_ticks = 0;
int32_t last_right_ticks = 0;
unsigned long last_send_time = 0;

void setup() {
  Serial.begin(115200);

  // CAN инициализация
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("❌ Ошибка reset()");
    while (1);
  }
  if (mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println("❌ Ошибка setBitrate()");
    while (1);
  }
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("❌ Ошибка setNormalMode()");
    while (1);
  }

  Serial.println("🚗 Arduino готово!");

  // стартовые значения моторов
  right_motor.set_pwmdir(120);
  left_motor.set_pwmdir(120);
}

void loop() {
  // === Чтение из CAN ===
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_dlc >= 8) {
      float v_lin, v_ang;
      memcpy(&v_lin, &canMsg.data[0], 4);
      memcpy(&v_ang, &canMsg.data[4], 4);

      Serial.print("📥 Получено: v_lin=");
      Serial.print(v_lin, 3);
      Serial.print("  v_ang=");
      Serial.println(v_ang, 3);

      set_velocity(v_lin, v_ang);
    }
  }

  // // === Обновление регуляторов каждые 10 мс ===
  static uint32_t t = millis();
  if (millis() - t >= 10) {
    t = millis();
    left_regulator.update();
    right_regulator.update();
  }

  // // === Отправка тиков только если изменились и прошло >= 30 мс ===
  unsigned long now = millis();
  int32_t left_ticks = left_enc.ticks;
  int32_t right_ticks = right_enc.ticks;
  bool changed = (left_ticks != last_left_ticks) || (right_ticks != last_right_ticks);
  if (changed && (now - last_send_time >= 40)) {
    canMsg.can_id  = 0x100;
    canMsg.can_dlc = 8;

    memcpy(&canMsg.data[0], &left_ticks, 4);
    memcpy(&canMsg.data[4], &right_ticks, 4);

    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
      Serial.print("📤 Тики отправлены: L=");
      Serial.print(left_ticks);
      Serial.print(" R=");
      Serial.println(right_ticks);
    } else {
      Serial.println("⚠️ Ошибка отправки CAN");
    }

    last_left_ticks = left_ticks;
    last_right_ticks = right_ticks;
    last_send_time = now;
  }
}

