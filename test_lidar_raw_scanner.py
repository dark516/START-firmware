#!/usr/bin/env python3

"""
Тестер нового LIDAR Raw Scanner - парсит raw данные и публикует в /scan

Использование:
    python3 test_lidar_raw_scanner.py [IP_ADDRESS]
"""

import subprocess
import sys
import os

def main():
    # IP адрес ESP32
    esp32_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.72"
    
    print("🔧 LIDAR Raw Scanner Tester")
    print("=" * 50)
    print(f"ESP32 IP: {esp32_ip}")
    print("=" * 50)
    
    # Путь к ROS2 workspace
    ros2_ws = "/home/egor/Desktop/project/ros2_ws"
    
    # Команда запуска
    cmd = [
        "bash", "-c",
        f"cd {ros2_ws} && "
        f"source install/setup.bash && "
        f"ros2 launch esp32_commander lidar_raw_scanner.launch.py "
        f"esp32_ip:={esp32_ip} "
        f"show_raw_data:=true "
        f"raw_data_format:=hex "
        f"bytes_per_point:=5"
    ]
    
    print("🚀 Запуск LIDAR Raw Scanner...")
    print("📊 Этот scanner будет:")
    print("   - Показывать raw LIDAR данные")
    print("   - Парсить их в LaserScan")
    print("   - Публиковать в топик /scan")
    print()
    print("🔍 В другом терминале проверьте:")
    print(f"   source {ros2_ws}/install/setup.bash")
    print("   ros2 topic list | grep scan")
    print("   ros2 topic hz /scan")
    print("   ros2 topic echo /scan --once")
    print()
    print("Нажмите Ctrl+C для остановки")
    print("-" * 50)
    
    try:
        # Запуск
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n🛑 Остановлено пользователем")
    except Exception as e:
        print(f"❌ Ошибка: {e}")

if __name__ == "__main__":
    main()