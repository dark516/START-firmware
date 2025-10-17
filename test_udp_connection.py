#!/usr/bin/env python3
"""
UDP Connection Test Script
Проверяет соединения с ESP32 по UDP
"""
import socket
import time
import threading

# Настройки (измени на свой ESP32 IP)
ESP32_IP = "192.168.125.222"
CMD_PORT = 3333     # Команды моторам и лидару
SENSOR_PORT = 3335  # Данные датчиков
LIDAR_PORT = 3334   # Данные лидара

def test_motor_commands():
    """Тест отправки команд моторам"""
    print("🚗 Тест команд моторам...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Отправляем тестовую команду
        command = "0.1 0.0"  # Вперед 0.1 м/с, поворот 0.0 рад/с
        sock.sendto(command.encode('utf-8'), (ESP32_IP, CMD_PORT))
        print(f"  ✅ Отправлена команда: {command}")
        
        time.sleep(1)
        
        # Останавливаем
        command = "0.0 0.0" 
        sock.sendto(command.encode('utf-8'), (ESP32_IP, CMD_PORT))
        print(f"  ✅ Отправлена команда: {command}")
        
    except Exception as e:
        print(f"  ❌ Ошибка: {e}")
    finally:
        sock.close()

def test_lidar_commands():
    """Тест команд лидару"""
    print("🔴 Тест команд лидару...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Команда SCAN
        scan_cmd = bytes([0xA5, 0x20])
        sock.sendto(scan_cmd, (ESP32_IP, CMD_PORT))
        print(f"  ✅ Отправлена команда SCAN лидару")
        
        time.sleep(0.5)
        
        # Команда GET_HEALTH
        health_cmd = bytes([0xA5, 0x52])
        sock.sendto(health_cmd, (ESP32_IP, CMD_PORT))
        print(f"  ✅ Отправлена команда HEALTH лидару")
        
    except Exception as e:
        print(f"  ❌ Ошибка: {e}")
    finally:
        sock.close()

def test_sensor_data():
    """Тест приема данных датчиков"""
    print("📡 Тест приема данных датчиков...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5.0)  # 5 секунд таймаут
    
    try:
        sock.bind(('', SENSOR_PORT))
        print(f"  🟢 Слушаем порт {SENSOR_PORT}...")
        
        packets_received = 0
        start_time = time.time()
        
        while time.time() - start_time < 5:
            try:
                data, addr = sock.recvfrom(1024)
                message = data.decode('utf-8', errors='ignore').strip()
                packets_received += 1
                
                if packets_received <= 3:  # Показываем первые 3 пакета
                    print(f"  📦 Пакет {packets_received}: '{message}' от {addr}")
                elif packets_received == 4:
                    print(f"  📦 ...")
                    
            except socket.timeout:
                break
                
        if packets_received > 0:
            rate = packets_received / 5.0
            print(f"  ✅ Получено {packets_received} пакетов за 5 сек (~{rate:.1f} Гц)")
        else:
            print(f"  ❌ Пакеты не получены!")
            
    except Exception as e:
        print(f"  ❌ Ошибка: {e}")
    finally:
        sock.close()

def test_lidar_data():
    """Тест приема данных лидара"""
    print("🔴 Тест приема данных лидара...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5.0)
    
    try:
        sock.bind(('', LIDAR_PORT))
        print(f"  🟢 Слушаем порт {LIDAR_PORT}...")
        
        packets_received = 0
        bytes_received = 0
        start_time = time.time()
        
        while time.time() - start_time < 5:
            try:
                data, addr = sock.recvfrom(4096)
                packets_received += 1
                bytes_received += len(data)
                
                if packets_received <= 2:  # Показываем первые 2 пакета
                    print(f"  📦 Пакет {packets_received}: {len(data)} байт от {addr}")
                elif packets_received == 3:
                    print(f"  📦 ...")
                    
            except socket.timeout:
                break
                
        if packets_received > 0:
            rate = bytes_received / 5.0 / 1024.0  # KB/s
            print(f"  ✅ Получено {packets_received} пакетов, {bytes_received} байт (~{rate:.1f} KB/s)")
        else:
            print(f"  ❌ Данные лидара не получены!")
            
    except Exception as e:
        print(f"  ❌ Ошибка: {e}")
    finally:
        sock.close()

def main():
    print("🚀 UDP Connection Test для ESP32 робота")
    print("=" * 50)
    print(f"ESP32 IP: {ESP32_IP}")
    print(f"Порты: CMD={CMD_PORT}, SENSOR={SENSOR_PORT}, LIDAR={LIDAR_PORT}")
    print()
    
    # Тест команд
    test_motor_commands()
    time.sleep(1)
    
    test_lidar_commands()
    time.sleep(1)
    
    # Тест данных (запускаем в отдельных потоках)
    print("\n📊 Тест приема данных (5 секунд каждый)...")
    
    sensor_thread = threading.Thread(target=test_sensor_data)
    lidar_thread = threading.Thread(target=test_lidar_data)
    
    sensor_thread.start()
    sensor_thread.join()
    
    lidar_thread.start() 
    lidar_thread.join()
    
    print("\n✅ Тест завершен!")
    print("\nЕсли данные не приходят:")
    print("1. Проверь IP адрес ESP32")
    print("2. Проверь что ESP32 подключен к WiFi")
    print("3. Проверь что ros2_ip в firmware указывает на этот компьютер")

if __name__ == "__main__":
    main()