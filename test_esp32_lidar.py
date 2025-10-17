#!/usr/bin/env python3
"""
Простой тест для проверки связи с ESP32 и получения LIDAR данных
"""
import socket
import time
import threading

def send_registration_commands():
    """Отправляем команды регистрации каждую секунду"""
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp32_ip = "192.168.125.222"
    cmd_port = 3333
    
    commands = [
        b'\xFF',                    # Простая регистрация
        b'\xA5\x25',               # LIDAR STOP  
        b'\xA5\x20',               # LIDAR START SCAN
        b'0.0 0.0',                # Команда моторов для регистрации
    ]
    
    print(f"📡 Отправляем команды на {esp32_ip}:{cmd_port}")
    
    for i, cmd in enumerate(commands):
        try:
            cmd_sock.sendto(cmd, (esp32_ip, cmd_port))
            print(f"  ✅ Команда #{i+1}: {len(cmd)} байт - {cmd}")
            time.sleep(1.0)
        except Exception as e:
            print(f"  ❌ Ошибка команды #{i+1}: {e}")
    
    cmd_sock.close()

def listen_for_lidar_data():
    """Слушаем LIDAR данные"""
    lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lidar_port = 3334
    
    try:
        lidar_sock.bind(('', lidar_port))
        lidar_sock.settimeout(1.0)
        print(f"🔍 Слушаем LIDAR на порту {lidar_port}")
        
        packet_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 10:  # 10 секунд
            try:
                data, addr = lidar_sock.recvfrom(4096)
                packet_count += 1
                print(f"📦 Пакет #{packet_count} от {addr}: {len(data)} байт")
                
                if packet_count == 1:
                    # Показать первый пакет в деталях
                    print(f"   Hex: {data[:50].hex()}")
                    print(f"   Первые 10 байт: {list(data[:10])}")
                
                if packet_count >= 5:
                    break
                    
            except socket.timeout:
                continue
                
        if packet_count == 0:
            print("❌ LIDAR данные не получены")
        else:
            print(f"✅ Получено {packet_count} LIDAR пакетов")
            
    except Exception as e:
        print(f"❌ Ошибка приема LIDAR: {e}")
    finally:
        lidar_sock.close()

def listen_for_sensor_data():
    """Слушаем данные датчиков"""
    sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sensor_port = 3335
    
    try:
        sensor_sock.bind(('', sensor_port))
        sensor_sock.settimeout(1.0)
        print(f"📊 Слушаем датчики на порту {sensor_port}")
        
        packet_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 10:  # 10 секунд
            try:
                data, addr = sensor_sock.recvfrom(1024)
                packet_count += 1
                print(f"🔬 Датчик #{packet_count} от {addr}: {len(data)} байт")
                
                if packet_count == 1:
                    print(f"   Hex: {data.hex()}")
                
                if packet_count >= 3:
                    break
                    
            except socket.timeout:
                continue
                
        if packet_count == 0:
            print("❌ Данные датчиков не получены")
        else:
            print(f"✅ Получено {packet_count} пакетов датчиков")
            
    except Exception as e:
        print(f"❌ Ошибка приема датчиков: {e}")
    finally:
        sensor_sock.close()

def main():
    print("╔════════════════════════════════════╗")
    print("║        ESP32 LIDAR TEST            ║") 
    print("╚════════════════════════════════════╝")
    
    # Запускаем потоки прослушивания
    lidar_thread = threading.Thread(target=listen_for_lidar_data, daemon=True)
    sensor_thread = threading.Thread(target=listen_for_sensor_data, daemon=True)
    
    lidar_thread.start()
    sensor_thread.start()
    
    # Ждем немного, затем отправляем команды
    time.sleep(1)
    send_registration_commands()
    
    # Ждем завершения потоков
    lidar_thread.join(timeout=12)
    sensor_thread.join(timeout=12)
    
    print("\n✅ Тест завершен")

if __name__ == "__main__":
    main()