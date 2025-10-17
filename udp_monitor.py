#!/usr/bin/env python3
"""
Мониторинг всех UDP портов ESP32
Поможет выяснить на какие порты приходят данные
"""
import socket
import time
import threading
import struct

ESP32_IP = "192.168.125.222"
CMD_PORT = 3333
SENSOR_PORT = 3335
LIDAR_PORT = 3334

def send_commands():
    """Отправляем команды для пробуждения ESP32"""
    print("📤 Отправляем команды ESP32...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        commands = [
            "0.0 0.0",  # Стоп
            "1.0 1.0",  # Движение
            bytes([0xA5, 0x20]),  # LIDAR SCAN
        ]
        
        for i, cmd in enumerate(commands):
            if isinstance(cmd, str):
                sock.sendto(cmd.encode('utf-8'), (ESP32_IP, CMD_PORT))
                print(f"  {i+1}. Мотор: '{cmd}'")
            else:
                sock.sendto(cmd, (ESP32_IP, CMD_PORT))
                print(f"  {i+1}. LIDAR: {[hex(b) for b in cmd]}")
            time.sleep(0.5)
            
    except Exception as e:
        print(f"Ошибка отправки: {e}")
    finally:
        sock.close()

def monitor_port(port, name, duration=15):
    """Мониторинг одного порта"""
    print(f"🟢 {name} (порт {port}) - запуск...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    try:
        sock.bind(('', port))
        
        packets = 0
        start_time = time.time()
        
        while time.time() - start_time < duration:
            try:
                data, addr = sock.recvfrom(4096)
                packets += 1
                
                print(f"📦 {name}: #{packets} - {len(data)}б от {addr[0]}:{addr[1]}")
                
                if packets <= 3:  # Показываем содержимое первых пакетов
                    if len(data) == 17:  # Возможно бинарный пакет датчиков
                        try:
                            values = struct.unpack('<LLLHHB', data)
                            print(f"    🔧 Бинарные данные: {values}")
                        except:
                            print(f"    📄 Hex: {data.hex()}")
                    else:
                        # Попробуем как текст
                        try:
                            text = data.decode('utf-8', errors='ignore').strip()
                            if text:
                                print(f"    📄 Текст: '{text}'")
                            else:
                                print(f"    📄 Hex: {data.hex()}")
                        except:
                            print(f"    📄 Hex: {data.hex()}")
                            
            except socket.timeout:
                continue
                
        print(f"✅ {name}: {packets} пакетов за {duration}с")
        
    except Exception as e:
        print(f"❌ {name}: {e}")
    finally:
        sock.close()

def main():
    print("🔍 UDP Монитор ESP32")
    print("=" * 40)
    
    # Отправляем команды
    send_commands()
    
    print("\n⏳ Ждем 2 сек...")
    time.sleep(2)
    
    # Запускаем мониторинг всех портов параллельно
    print("\n🚀 Запуск мониторинга...")
    
    threads = []
    ports_info = [
        (CMD_PORT, "КОМАНДЫ"),
        (SENSOR_PORT, "ДАТЧИКИ"), 
        (LIDAR_PORT, "LIDAR"),
        (8888, "АЛЬТ-1"),  # Возможные альтернативные порты
        (9999, "АЛЬТ-2"),
    ]
    
    for port, name in ports_info:
        thread = threading.Thread(target=monitor_port, args=(port, name, 15))
        thread.start()
        threads.append(thread)
        time.sleep(0.1)  # Небольшая задержка между запусками
    
    # Периодически отправляем команды для поддержания связи
    def keep_sending():
        for i in range(5):  # 5 раз с интервалом 3 сек
            time.sleep(3)
            send_commands()
    
    keep_thread = threading.Thread(target=keep_sending)
    keep_thread.start()
    
    # Ждем завершения всех потоков
    for thread in threads:
        thread.join()
    
    keep_thread.join()
    
    print("\n" + "="*40)
    print("✅ Мониторинг завершен")

if __name__ == "__main__":
    main()