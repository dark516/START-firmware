#!/usr/bin/env python3
"""
Исправленный приемник LIDAR данных
Учитывает результаты анализа сетевого трафика
"""
import socket
import time
import threading
import struct
from datetime import datetime

ESP32_IP = "192.168.125.222"
LIDAR_PORT = 3334  # Порт откуда ESP32 ОТПРАВЛЯЕТ данные LIDAR
LOCAL_LIDAR_PORT = 3334  # Порт на котором мы СЛУШАЕМ

# Статистика
class Stats:
    def __init__(self):
        self.total_packets = 0
        self.total_bytes = 0
        self.start_time = time.time()
        self.last_print = 0
        
    def update(self, packet_size):
        self.total_packets += 1
        self.total_bytes += packet_size
        
        # Печатаем статистику каждые 2 секунды
        if time.time() - self.last_print > 2.0:
            duration = time.time() - self.start_time
            pps = self.total_packets / duration
            bps = self.total_bytes / duration
            
            print(f"\n📊 Статистика LIDAR:")
            print(f"  📦 {self.total_packets} пакетов ({pps:.1f} pакетов/с)")
            print(f"  📈 {self.total_bytes} байт ({bps/1024:.1f} KB/s)")
            print(f"  📏 Средний размер: {self.total_bytes/self.total_packets:.1f} байт")
            
            self.last_print = time.time()

def parse_lidar_packet(data):
    """Попытка разбора пакета LIDAR"""
    try:
        # Показываем hex данные для первых пакетов
        hex_data = data.hex()
        
        # Попробуем найти известные паттерны
        if len(data) >= 12:  # Основной размер пакетов по tcpdump
            # Возможная структура: angle, distance, quality, timestamp
            values = struct.unpack('<HHH', data[:6]) if len(data) >= 6 else None
            if values:
                return f"A:{values[0]} D:{values[1]} Q:{values[2]} [{hex_data}]"
        
        return f"Raw[{len(data)}]: {hex_data}"
        
    except Exception as e:
        return f"Parse error: {e} - {data.hex()}"

def receive_lidar_data():
    """Основной приемник данных LIDAR"""
    print(f"🔴 Запуск LIDAR приемника на порту {LOCAL_LIDAR_PORT}")
    
    # Создаем сокет с увеличенными буферами
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Увеличиваем буферы для обработки высокоскоростного потока
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    sock.settimeout(0.1)  # Короткий таймаут для быстрого цикла
    
    stats = Stats()
    
    try:
        # Привязываемся к порту
        sock.bind(('', LOCAL_LIDAR_PORT))
        print(f"✅ Слушаем UDP порт {LOCAL_LIDAR_PORT}")
        
        packet_count = 0
        
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                
                packet_count += 1
                stats.update(len(data))
                
                # Показываем первые 5 пакетов
                if packet_count <= 5:
                    parsed = parse_lidar_packet(data)
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    print(f"📦 [{timestamp}] #{packet_count} от {addr[0]}:{addr[1]}")
                    print(f"    {parsed}")
                elif packet_count == 6:
                    print(f"📦 ... (продолжается прием, всего: {packet_count})")
                
                # Простая проверка адреса отправителя
                if addr[0] != ESP32_IP:
                    print(f"⚠️ Неожиданный отправитель: {addr[0]} (ожидался {ESP32_IP})")
                
            except socket.timeout:
                # Таймаут - это нормально, продолжаем цикл
                continue
                
            except Exception as e:
                print(f"❌ Ошибка приема: {e}")
                break
                
    except Exception as e:
        print(f"❌ Ошибка инициализации: {e}")
    finally:
        sock.close()
        print(f"\n📊 Финальная статистика:")
        print(f"  📦 Всего получено: {stats.total_packets} пакетов")
        print(f"  📈 Общий объем: {stats.total_bytes} байт")

def send_activation_commands():
    """Отправляем команды активации ESP32"""
    print("📤 Активация ESP32...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        commands = [
            ("0.0 0.0", "Стоп моторов"),
            (bytes([0xA5, 0x20]), "LIDAR сканирование"),
            (bytes([0xA5, 0x21]), "LIDAR старт"),  # Дополнительные команды
        ]
        
        for cmd, desc in commands:
            if isinstance(cmd, str):
                sock.sendto(cmd.encode('utf-8'), (ESP32_IP, 3333))
            else:
                sock.sendto(cmd, (ESP32_IP, 3333))
            print(f"  ✅ {desc}")
            time.sleep(0.3)
            
    except Exception as e:
        print(f"  ❌ Ошибка активации: {e}")
    finally:
        sock.close()

def main():
    print("🚀 Исправленный LIDAR приемник")
    print("=" * 50)
    
    # По анализу tcpdump знаем что ESP32 уже активен
    print("ℹ️ По анализу трафика ESP32 уже активен и отправляет данные")
    
    # Отправляем команды активации
    send_activation_commands()
    
    print("\n⏳ Запуск приема через 1 секунду...")
    time.sleep(1)
    
    # Запускаем прием
    try:
        receive_lidar_data()
    except KeyboardInterrupt:
        print("\n⏹️ Остановлено пользователем")
        
    print("\n✅ Завершено")

if __name__ == "__main__":
    main()