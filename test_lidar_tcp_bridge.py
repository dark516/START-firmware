#!/usr/bin/env python3

"""
Тест LIDAR TCP Bridge с отображением raw данных и публикацией в /scan

Использование:
    python3 test_lidar_tcp_bridge.py

Проверяет:
- TCP подключение к ESP32
- Получение raw LIDAR данных  
- Парсинг данных в LaserScan
- Публикацию в топик /scan
"""

import subprocess
import sys
import time
import os

def check_ros2_setup():
    """Проверка настройки ROS2"""
    print("🔍 Проверка ROS2...")
    
    # Проверяем source setup
    ros2_ws_path = "/home/egor/Desktop/project/ros2_ws"
    setup_file = f"{ros2_ws_path}/install/setup.bash"
    
    if not os.path.exists(setup_file):
        print(f"❌ Файл setup не найден: {setup_file}")
        print("   Запустите: cd ros2_ws && colcon build")
        return False
    
    print(f"✅ ROS2 workspace: {ros2_ws_path}")
    return True

def test_esp32_connection():
    """Проверка подключения к ESP32"""
    print("\n🔗 Проверка подключения к ESP32...")
    
    esp32_ip = "192.168.125.222"
    esp32_port = 3333
    
    try:
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        result = sock.connect_ex((esp32_ip, esp32_port))
        sock.close()
        
        if result == 0:
            print(f"✅ ESP32 доступен: {esp32_ip}:{esp32_port}")
            return True
        else:
            print(f"❌ ESP32 недоступен: {esp32_ip}:{esp32_port}")
            return False
            
    except Exception as e:
        print(f"❌ Ошибка подключения: {e}")
        return False

def run_lidar_bridge():
    """Запуск LIDAR TCP Bridge"""
    print("\n🚀 Запуск LIDAR TCP Bridge...")
    print("   (Нажмите Ctrl+C для остановки)")
    
    ros2_ws_path = "/home/egor/Desktop/project/ros2_ws"
    
    # Команда для запуска
    cmd = [
        "bash", "-c", 
        f"cd {ros2_ws_path} && "
        f"source install/setup.bash && "
        f"ros2 launch esp32_commander lidar_tcp_raw.launch.py "
        f"esp32_ip:=192.168.125.222 "
        f"esp32_port:=3333 "
        f"show_raw_data:=true "
        f"raw_data_format:=both "
        f"max_raw_bytes:=128"
    ]
    
    try:
        # Запускаем процесс
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        
        print("📡 LIDAR Bridge запущен...")
        print("📊 Ожидайте raw данные и LaserScan сообщения...")
        print("🔍 Для просмотра топиков откройте новый терминал и запустите:")
        print("   ros2 topic list")
        print("   ros2 topic hz /scan")
        print("   ros2 topic echo /scan --once")
        
        # Читаем вывод в реальном времени
        while True:
            line = process.stdout.readline()
            if line:
                print(line.rstrip())
            elif process.poll() is not None:
                break
                
    except KeyboardInterrupt:
        print("\n🛑 Остановка LIDAR Bridge...")
        if process:
            process.terminate()
            process.wait()
    except Exception as e:
        print(f"❌ Ошибка запуска: {e}")

def show_usage():
    """Показ инструкций по использованию"""
    print("\n" + "="*60)
    print("📖 ИНСТРУКЦИИ ПО ИСПОЛЬЗОВАНИЮ")
    print("="*60)
    print("1. 🔌 Убедитесь что ESP32 подключен и работает:")
    print("   - IP: 192.168.125.222")
    print("   - Порт: 3333")
    print("   - LIDAR подключен к ESP32")
    print()
    print("2. 🏗️ Соберите ROS2 пакет (если еще не собран):")
    print("   cd /home/egor/Desktop/project/ros2_ws")
    print("   colcon build --packages-select esp32_commander")
    print()
    print("3. 🚀 Запустите этот скрипт:")
    print("   python3 test_lidar_tcp_bridge.py")
    print()
    print("4. 📊 В другом терминале проверьте топики:")
    print("   source /home/egor/Desktop/project/ros2_ws/install/setup.bash")
    print("   ros2 topic list | grep scan")
    print("   ros2 topic hz /scan")
    print("   ros2 topic echo /scan --once")
    print()
    print("5. 🛠️ Если не работает, проверьте:")
    print("   - Подключение ESP32 (ping 192.168.125.222)")
    print("   - Прошивку ESP32 (esp32_lidar_tcp.cpp)")
    print("   - Команды LIDAR (START/STOP через Serial Monitor)")
    print("="*60)

def main():
    print("🔧 LIDAR TCP Bridge Tester")
    print("=" * 40)
    
    # Проверяем ROS2
    if not check_ros2_setup():
        show_usage()
        return 1
    
    # Проверяем ESP32
    if not test_esp32_connection():
        print("\n⚠️ ESP32 недоступен. Проверьте:")
        print("   1. IP адрес ESP32: ping 192.168.125.222")
        print("   2. Прошивку ESP32: esp32_lidar_tcp.cpp")
        print("   3. Serial Monitor ESP32 для диагностики")
        show_usage()
        return 1
    
    # Запускаем bridge
    try:
        run_lidar_bridge()
    except Exception as e:
        print(f"❌ Критическая ошибка: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())