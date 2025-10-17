# 📡 LIDAR TCP Bridge с поддержкой /scan

## Что изменилось

Обновлен `lidar_tcp_bridge.py` для:
1. **📊 Отображения raw LIDAR данных** (hex/ascii форматы)
2. **🔄 Парсинга RPLiDAR протокола** из raw TCP данных
3. **📡 Публикации в топик `/scan`** (sensor_msgs/LaserScan)

## Архитектура

```
ESP32 (esp32_lidar_tcp.cpp) 
    ↓ TCP Port 3333
    ↓ Raw LIDAR bytes
lidar_tcp_bridge.py
    ↓ Парсинг RPLiDAR протокола
    ↓ Публикация LaserScan
ROS2 Topic: /scan
```

## Быстрый старт

### 1. Сборка
```bash
cd /home/egor/Desktop/project/ros2_ws
colcon build --packages-select esp32_commander
```

### 2. Запуск
```bash
# Простой запуск
python3 /home/egor/Desktop/project/test_lidar_tcp_bridge.py

# Или через ROS2 launch
source ros2_ws/install/setup.bash
ros2 launch esp32_commander lidar_tcp_raw.launch.py
```

### 3. Проверка работы
```bash
# В другом терминале
source ros2_ws/install/setup.bash
ros2 topic list | grep scan
ros2 topic hz /scan
ros2 topic echo /scan --once
```

## Параметры launch файла

```bash
ros2 launch esp32_commander lidar_tcp_raw.launch.py \
  esp32_ip:=192.168.125.222 \
  esp32_port:=3333 \
  show_raw_data:=true \
  raw_data_format:=both \
  max_raw_bytes:=128
```

### Параметры:
- `esp32_ip` - IP адрес ESP32 (по умолчанию: 192.168.125.222)
- `esp32_port` - TCP порт ESP32 (по умолчанию: 3333)  
- `show_raw_data` - показывать raw данные (true/false)
- `raw_data_format` - формат: hex, ascii, both
- `max_raw_bytes` - максимум байт для отображения
- `frame_id` - frame_id для LaserScan (по умолчанию: laser)
- `angle_min/max` - диапазон углов в радианах
- `range_min/max` - диапазон дальности в метрах
- `scan_frequency` - частота сканов в Hz

## Логи и диагностика

### Успешная работа:
```
✅ LIDAR TCP Bridge для RAW данных создан
📡 LaserScan параметры:
   frame_id: laser
   angle: -180.0° to 180.0°
   range: 0.15m to 12.0m
   frequency: 10.0Hz
🔗 Подключено к ESP32 (192.168.125.222:3333)
📤 Команда LIDAR: START

╔══════════════ RAW LIDAR DATA ═══════════════╗
║ Размер пакета: 512 байт
║ HEX:
║   A5 5A 81 3C 12 34 56 78 9A BC DE F0...
║ ASCII:
║   ...<.4Vx....
║ Паттерны: A5 5A at offset 0
║ Частые байты: 0xA5(12), 0x5A(8), 0x81(4)
╚══════════════════════════════════════════════╝

📡 LaserScan #1: 287/360 точек, угол: -180° - 180°
```

### Статистика работы:
```
╔═════════════ RAW LIDAR STATS ════════════════╗
║ Время работы: 30.5 сек
║ TCP пакетов: 156 (5.1/сек)
║ Сырых байт: 45632 (1496 байт/сек)
║ Отображено чанков: 30 (0.98/сек)
║ 📡 LaserScans: 25 (0.82/сек)
║ 📊 Точек на скан: 234 среднем
║ Размер буфера: 2.3 KB
║ ✅ RAW LIDAR поток активен
╚══════════════════════════════════════════════╝
```

## Проблемы и решения

### ESP32 недоступен
```bash
# Проверьте подключение
ping 192.168.125.222

# Проверьте прошивку ESP32
# Загрузите esp32_lidar_tcp.cpp на ESP32
# Подключитесь к Serial Monitor (115200 baud)
```

### Нет LIDAR данных
```bash
# В Serial Monitor ESP32 отправьте команды:
START   # Начать сканирование
STOP    # Остановить
HEALTH  # Проверить состояние
RAW     # Переключить отображение raw данных
```

### Низкое качество сканов
- Проверьте подключение LIDAR к ESP32
- Убедитесь что LIDAR получает питание
- Проверьте правильность подключения TX/RX пинов
- В коде ESP32: RX=11, TX=12, Baud=460800

## Файлы проекта

- `ros2_ws/src/esp32_commander/esp32_commander/lidar_tcp_bridge.py` - ROS2 bridge
- `ros2_ws/src/esp32_commander/launch/lidar_tcp_raw.launch.py` - Launch файл
- `ros2_ws/esp32_lidar_tcp.cpp` - Прошивка ESP32
- `test_lidar_tcp_bridge.py` - Тестовый скрипт