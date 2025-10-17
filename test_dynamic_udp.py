#!/usr/bin/env python3
"""
Тест динамического UDP подключения
Показывает как ESP32 автоматически определяет IP клиента
"""
import socket
import time
import threading

ESP32_IP = "192.168.125.222"
CMD_PORT = 3333
SENSOR_PORT = 3335
LIDAR_PORT = 3334

def send_commands():
    """Отправляем команды, чтобы ESP32 запомнил наш IP"""
    print("🚀 Отправка команд для регистрации IP...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Отправляем несколько команд
        commands = [
            "0.0 0.0",  # Стоп
            bytes([0xA5, 0x20]),  # LIDAR SCAN
            "0.1 0.0",  # Вперед
            "0.0 0.0",  # Стоп
        ]
        
        for i, cmd in enumerate(commands):
            if isinstance(cmd, str):
                sock.sendto(cmd.encode('utf-8'), (ESP32_IP, CMD_PORT))
                print(f"  📤 {i+1}. Отправлена команда мотора: '{cmd}'")
            else:
                sock.sendto(cmd, (ESP32_IP, CMD_PORT))
                print(f"  📤 {i+1}. Отправлена команда LIDAR: {list(cmd)}")
            
            time.sleep(0.5)
        
        print("✅ Команды отправлены. ESP32 должен запомнить наш IP.")
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        sock.close()

def listen_for_data():
    """Слушаем данные от ESP32"""
    def listen_sensors():
        print("📡 Слушаем данные датчиков...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        
        try:
            sock.bind(('', SENSOR_PORT))
            
            for i in range(10):  # 10 попыток
                try:
                    data, addr = sock.recvfrom(1024)
                    message = data.decode('utf-8', errors='ignore')
                    print(f"  📦 Датчики: '{message.strip()}' от {addr}")
                    return True
                except socket.timeout:
                    continue
            
            print("  ❌ Данные датчиков не получены")
            return False
        except Exception as e:
            print(f"  ❌ Ошибка датчиков: {e}")
            return False
        finally:
            sock.close()
    
    def listen_lidar():
        print("🔴 Слушаем данные LIDAR...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2.0)
        
        try:
            sock.bind(('', LIDAR_PORT))
            
            for i in range(10):  # 10 попыток
                try:
                    data, addr = sock.recvfrom(4096)
                    print(f"  📦 LIDAR: {len(data)} байт от {addr}")
                    return True
                except socket.timeout:
                    continue
            
            print("  ❌ Данные LIDAR не получены")
            return False
        except Exception as e:
            print(f"  ❌ Ошибка LIDAR: {e}")
            return False
        finally:
            sock.close()
    
    # Запускаем прослушивание в потоках
    sensor_thread = threading.Thread(target=listen_sensors)
    lidar_thread = threading.Thread(target=listen_lidar)
    
    sensor_thread.start()
    lidar_thread.start()
    
    sensor_thread.join()
    lidar_thread.join()

def main():
    print("🔄 Тест динамического UDP подключения")
    print("=" * 45)
    print(f"ESP32: {ESP32_IP}")
    print()
    
    # 1. Отправляем команды для регистрации IP
    send_commands()
    
    print()
    print("⏳ Ждем 2 секунды...")
    time.sleep(2)
    print()
    
    # 2. Слушаем данные
    listen_for_data()
    
    print()
    print("✅ Тест завершен!")
    print()
    print("Как это работает:")
    print("1. ESP32 получает первую команду и запоминает IP отправителя")
    print("2. Все последующие данные отправляются на этот IP")
    print("3. Не нужно прописывать IP в коде ESP32!")

if __name__ == "__main__":
    main()