#!/usr/bin/env python3
"""
Детальный тест UDP соединения
Показывает подробную информацию о принимаемых данных
"""
import socket
import time
import threading
import struct

ESP32_IP = "192.168.125.222"
CMD_PORT = 3333
SENSOR_PORT = 3335
LIDAR_PORT = 3334

def send_initial_commands():
    """Отправляем команды для регистрации IP"""
    print("🔌 Регистрация IP адреса на ESP32...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Команды для регистрации
        commands = [
            "0.0 0.0",  # Стоп моторов
            bytes([0xA5, 0x20]),  # LIDAR SCAN  
        ]
        
        for cmd in commands:
            if isinstance(cmd, str):
                sock.sendto(cmd.encode('utf-8'), (ESP32_IP, CMD_PORT))
                print(f"  📤 Команда мотора: '{cmd}'")
            else:
                sock.sendto(cmd, (ESP32_IP, CMD_PORT))
                print(f"  📤 LIDAR команда: {[hex(b) for b in cmd]}")
            time.sleep(0.2)
        
        print("  ✅ IP зарегистрирован")
        
    except Exception as e:
        print(f"  ❌ Ошибка: {e}")
    finally:
        sock.close()

def test_sensor_data():
    """Детальный тест данных датчиков"""
    print("\n📡 === ТЕСТ ДАННЫХ ДАТЧИКОВ ===")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    try:
        sock.bind(('', SENSOR_PORT))
        print(f"🟢 Слушаем порт {SENSOR_PORT}...")
        
        packets = 0
        start_time = time.time()
        
        while time.time() - start_time < 10:  # 10 секунд
            try:
                data, addr = sock.recvfrom(1024)
                message = data.decode('utf-8', errors='ignore').strip()
                packets += 1
                
                if packets <= 5:  # Показываем первые 5
                    print(f"  📦 #{packets}: '{message}' от {addr[0]}:{addr[1]}")
                elif packets == 6:
                    print(f"  📦 ... (продолжается)")
                
                # Парсим данные
                try:
                    parts = message.split()
                    if len(parts) >= 4:
                        left, right, yaw, accel = parts[:4]
                        if packets % 50 == 0:  # Каждый 50-й пакет
                            print(f"  📊 #{packets}: L={left} R={right} Yaw={yaw}° Acc={accel}")
                except:
                    pass
                    
            except socket.timeout:
                if packets == 0:
                    print("  ⏰ Таймаут - данных нет")
                continue
        
        if packets > 0:
            rate = packets / 10.0
            print(f"\n  ✅ Получено {packets} пакетов за 10 сек")
            print(f"  📈 Частота: {rate:.1f} Гц")
        else:
            print(f"\n  ❌ Данные датчиков НЕ получены!")
            
    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        sock.close()

def test_lidar_data():
    """Детальный тест данных LIDAR"""
    print("\n🔴 === ТЕСТ ДАННЫХ LIDAR ===")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    try:
        sock.bind(('', LIDAR_PORT))
        print(f"🟢 Слушаем порт {LIDAR_PORT}...")
        
        packets = 0
        total_bytes = 0
        start_time = time.time()
        
        while time.time() - start_time < 10:  # 10 секунд
            try:
                data, addr = sock.recvfrom(4096)
                packets += 1
                total_bytes += len(data)
                
                if packets <= 3:  # Показываем первые 3
                    print(f"  📦 #{packets}: {len(data)} байт от {addr[0]}:{addr[1]}")
                    # Показываем первые байты
                    preview = ' '.join([f"{b:02X}" for b in data[:min(16, len(data))]])
                    if len(data) > 16:
                        preview += "..."
                    print(f"      Данные: {preview}")
                elif packets == 4:
                    print(f"  📦 ... (продолжается)")
                
                if packets % 100 == 0:  # Каждый 100-й пакет
                    rate = total_bytes / (time.time() - start_time) / 1024
                    print(f"  📊 #{packets}: {rate:.1f} KB/s")
                    
            except socket.timeout:
                if packets == 0:
                    print("  ⏰ Таймаут - данных нет")
                continue
        
        if packets > 0:
            duration = time.time() - start_time
            rate = total_bytes / duration / 1024.0
            pps = packets / duration
            print(f"\n  ✅ Получено {packets} пакетов, {total_bytes} байт")
            print(f"  📈 Скорость: {rate:.1f} KB/s, {pps:.1f} пакетов/с")
            print(f"  📏 Средний размер пакета: {total_bytes/packets:.0f} байт")
        else:
            print(f"\n  ❌ Данные LIDAR НЕ получены!")
            
    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        sock.close()

def main():
    print("🔬 Детальный UDP тест для ESP32 робота")
    print("=" * 50)
    print(f"ESP32 IP: {ESP32_IP}")
    print(f"Порты: CMD={CMD_PORT}, SENSOR={SENSOR_PORT}, LIDAR={LIDAR_PORT}")
    
    # 1. Регистрируем IP
    send_initial_commands()
    
    print("\n⏳ Ждем 2 секунды для стабилизации...")
    time.sleep(2)
    
    # 2. Тестируем в параллельных потоках
    print("\n🧪 Запуск параллельного тестирования...")
    
    sensor_thread = threading.Thread(target=test_sensor_data)
    lidar_thread = threading.Thread(target=test_lidar_data)
    
    sensor_thread.start()
    time.sleep(0.5)  # Небольшая задержка между стартами
    lidar_thread.start()
    
    sensor_thread.join()
    lidar_thread.join()
    
    print("\n" + "="*50)
    print("✅ Тестирование завершено!")
    print()
    print("💡 Что означают результаты:")
    print("• Датчики должны давать ~50 Гц (50 пакетов/сек)")
    print("• LIDAR должен давать ~10-50 KB/s")
    print("• Если данных нет - проверь прошивку ESP32")

if __name__ == "__main__":
    main()