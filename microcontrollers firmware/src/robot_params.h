#pragma once
#include <math.h>
#include <Arduino.h>
constexpr float DT = 0.01f; // Важно: Добавить эту строку!
// Железо роботаd
// Параметры колесной базы
constexpr float WHEEL_DIAMETER = 0.04f;     // [m]
constexpr int TICKS_PER_REV = 1066;           // [ticks/rev]
constexpr float WHEEL_BASE = 0.1665f;         // [m]

// Ограничения
constexpr float MAX_LIN_SPEED = 0.4863f;        // [m/s]
constexpr float MAX_ANG_SPEED = 5.4f;        // [rad/s]
constexpr float MAX_LIN_ACCEL = 5.0f;        // [m/s²]
constexpr float MAX_ANG_ACCEL = 5.0f;        // [rad/s²]

// Расчетные константы
constexpr float METERS_PER_TICK = 0.000117824f;
constexpr float TICKS_PER_METER = 1.0f / METERS_PER_TICK;
constexpr int MAX_DELTA_TICKS = static_cast<int>(MAX_LIN_SPEED * TICKS_PER_METER);      