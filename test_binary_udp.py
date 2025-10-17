#!/usr/bin/env python3
"""
Тест бинарных UDP пакетов
Работает с новыми бинарными структурами ESP32
"""
import socket
import time
import threading
import struct

ESP32_IP = "192.168.125.222"
CMD_PORT = 3333
SENSOR_PORT = 3335
LIDAR_PORT = 3334

# Бинарная структура датчиков (должна совпадать с ESP32)
SENSOR_PACKET_FORMAT = '<LLLHHB'  # Little-endian: timestamp, left_ticks, right_ticks, yaw, accel_x, packet_id
SENSOR_PACKET_SIZE = struct.calcsize(SENSOR_PACKET_FORMAT)

def send_initial_commands():
    """Отправляем команды для регистрации IP"""
    print("🔌 Регистрация IP адреса на ESP32...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
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

def test_binary_sensor_data():
    """Тест бинарных пакетов датчиков"""
    print(f"\n📦 === ТЕСТ БИНАРНЫХ ПАКЕТОВ ДАТЧИКОВ ===")
    print(f"Размер пакета: {SENSOR_PACKET_SIZE} байт")
    print(f"Формат: {SENSOR_PACKET_FORMAT}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)
    
    try:
        sock.bind(('', SENSOR_PORT))
        print(f"🟢 Слушаем порт {SENSOR_PORT}...")
        
        packets = 0
        start_time = time.time()
        last_packet_id = None
        
        while time.time() - start_time < 10:  # 10 секунд
            try:
                data, addr = sock.recvfrom(1024)
                
                if len(data) == SENSOR_PACKET_SIZE:
                    # Бинарный пакет
                    timestamp, left_ticks, right_ticks, yaw_raw, accel_raw, packet_id = struct.unpack(SENSOR_PACKET_FORMAT, data)
                    
                    # Преобразуем данные
                    yaw_deg = yaw_raw / 10.0
                    x_accel = accel_raw / 1000.0
                    
                    packets += 1
                    
                    if packets <= 5:  # Показываем первые 5
                        print(f"  📦 #{packet_id}: L={left_ticks} R={right_ticks} Yaw={yaw_deg:.1f}° Acc={x_accel:.3f}m/s²")
                    elif packets == 6:
                        print(f"  📦 ... (продолжается)")
                    
                    # Проверяем пропуски пакетов
                    if last_packet_id is not None:
                        expected = (last_packet_id + 1) % 256
                        if packet_id != expected:
                            print(f"  ⚠️ Пропуск пакета! Ожидался #{expected}, получен #{packet_id}")
                    
                    last_packet_id = packet_id
                    
                    if packets % 50 == 0:  # Каждый 50-й пакет
                        print(f"  📊 #{packet_id}: {packets} пакетов получено")
                else:
                    # Возможно текстовый пакет (старый формат)
                    try:
                        text = data.decode('utf-8', errors='ignore').strip()
                        print(f"  📄 Текстовый пакет: '{text}' (размер: {len(data)} байт)")
                    except:
                        print(f"  ❓ Неизвестный пакет размером {len(data)} байт")
                    
            except socket.timeout:
                if packets == 0:
                    print("  ⏰ Таймаут - пакетов нет")
                continue
        
        if packets > 0:
            rate = packets / 10.0
            print(f"\n  ✅ Получено {packets} бинарных пакетов за 10 сек")
            print(f"  📈 Частота: {rate:.1f} пакетов/с")
            print(f"  📏 Размер каждого пакета: {SENSOR_PACKET_SIZE} байт")
        else:
            print(f"\n  ❌ Бинарные пакеты НЕ получены!")
            
    except Exception as e:
        print(f"❌ Ошибка: {e}")
    finally:
        sock.close()

def test_lidar_data():
    """Тест данных LIDAR"""
    print(f"\n🔴 === ТЕСТ ДАННЫХ LIDAR ===")
    
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
                
                if packets % 50 == 0:  # Каждый 50-й пакет
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
    print("🚀 Тест бинарных UDP пакетов ESP32")
    print("=" * 50)
    print(f"ESP32 IP: {ESP32_IP}")
    print(f"Порты: CMD={CMD_PORT}, SENSOR={SENSOR_PORT}, LIDAR={LIDAR_PORT}")
    print(f"Бинарные пакеты: {SENSOR_PACKET_SIZE} байт")
    
    # 1. Регистрируем IP
    send_initial_commands()
    
    print("\n⏳ Ждем 2 секунды для стабилизации...")
    time.sleep(2)
    
    # 2. Тестируем в параллельных потоках
    print("\n🧪 Запуск параллельного тестирования...")
    
    sensor_thread = threading.Thread(target=test_binary_sensor_data)
    lidar_thread = threading.Thread(target=test_lidar_data)
    
    sensor_thread.start()
    time.sleep(0.5)  # Небольшая задержка между стартами
    lidar_thread.start()
    
    sensor_thread.join()
    lidar_thread.join()
    
    print("\n" + "="*50)
    print("✅ Тестирование завершено!")
    print()
    print("💡 Преимущества бинарных пакетов:")
    print(f"• Фиксированный размер: {SENSOR_PACKET_SIZE} байт")
    print("• Быстрее парсинга строк")
    print("• Надежнее передача")
    print("• Меньше сетевого трафика")

if __name__ == "__main__":
    main()