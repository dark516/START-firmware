#include "motor_regulator.h"
#include "ros2_communication.hpp"

// Форвард-декларации
class Regulator;
extern Regulator right_regulator;

// В файле arduino_code.ino

Motor right_motor(8, 9);

Encoder right_enc(3, 5, []{ right_regulator.encoder.encoder_int(); }, true); // INT1 (pin 3), B=12, invert=false

// Остальной код остается без изменений

// Создаем PID-регуляторы
PID right_pid(4.0, 0.2, 0.04, 100);

// Создаем регуляторы, передавая созданные объекты
Regulator right_regulator(right_motor, right_enc, right_pid);

void setup() {
  Serial.begin(115200);

    right_regulator.set_speed(1.0);

  static uint32_t t = millis() + 5000;
    
    while (millis() < t)    {
      Serial.println(right_enc.ticks);
        right_regulator.update();
        delay(10);    
    }
    

}

void loop() { }