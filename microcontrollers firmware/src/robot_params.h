#pragma once
#include <math.h>
#include <Arduino.h>

constexpr float DT = 0.01f; // Важно: Добавить эту строку!

// Железо робота
// Параметры колесной базы
constexpr float WHEEL_DIAMETER = 0.04f;     // [m]
constexpr int TICKS_PER_REV = 1066;           // [ticks/rev]
constexpr float WHEEL_BASE = 0.1665f;         // [m]

// Ограничения
constexpr float MAX_LIN_SPEED = 0.37f;        // [m/s]
constexpr float MAX_ANG_SPEED = 4.44f;        // [rad/s] - Рассчитано: (2 * MAX_LIN_SPEED) / WHEEL_BASE
constexpr float MAX_LIN_ACCEL = 1.0f;         // [m/s²] - Оценка для робота такого размера
constexpr float MAX_ANG_ACCEL = 4.44f;        // [rad/s²] - Оценка, основанная на MAX_ANG_SPEED

// Расчетные константы
constexpr float METERS_PER_TICK = (M_PI * WHEEL_DIAMETER) / TICKS_PER_REV; // [m/tick]
constexpr float TICKS_PER_METER = 1.0f / METERS_PER_TICK;               // [ticks/m]
constexpr int MAX_DELTA_TICKS = static_cast<int>(MAX_LIN_SPEED * TICKS_PER_METER); // [ticks/DT]