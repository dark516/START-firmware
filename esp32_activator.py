#!/usr/bin/env python3
"""
ESP32 активатор с мониторингом всех портов
Отправляет команды и слушает ответы одновременно
"""
import socket
import time
import threading
from datetime import datetime

ESP32_IP = "192.168.125.222"
CMD_PORT = 3333
SENSOR_PORT = 3335  
LIDAR_PORT = 3334

class MultiPortMonitor:
    def __init__(self):
        self.monitoring = True
        self.stats = {}
        
    def monitor_port(self, port, name):
        """Мониторинг одного порта"""
        print(f"🟢 Запуск мониторинга {name} (порт {port})")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)
        
        self.stats[name] = {"packets": 0, "bytes": 0}
        
        try:
            sock.bind(('', port))
            
            while self.monitoring:
                try:
                    data, addr = sock.recvfrom(4096)
                    
                    self.stats[name]["packets"] += 1
                    self.stats[name]["bytes"] += len(data)
                    
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    
                    if self.stats[name]["packets"] <= 3:
                        print(f"📦 [{timestamp}] {name}: {len(data)}б от {addr[0]}:{addr[1]}")
                        print(f"    Hex: {data.hex()}")
                    elif self.stats[name]["packets"] == 4:
                        print(f"📦 {name}: ... (продолжается)")
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"❌ {name}: {e}")
                    break
                    
        except Exception as e:
            print(f"❌ Не удалось привязаться к порту {port} ({name}): {e}")
        finally:
            sock.close()
            
    def start_monitoring(self):
        """Запуск мониторинга всех портов"""
        ports = [
            (CMD_PORT, "КОМАНДЫ"),
            (SENSOR_PORT, "ДАТЧИКИ"),  
            (LIDAR_PORT, "LIDAR"),
            (8888, "АЛЬТ-1"),
            (9999, "АЛЬТ-2")
        ]
        
        threads = []
        for port, name in ports:
            thread = threading.Thread(target=self.monitor_port, args=(port, name))
            thread.daemon = True
            thread.start()
            threads.append(thread)
            time.sleep(0.1)
            
        return threads
    
    def stop_monitoring(self):
        """Остановка мониторинга"""
        self.monitoring = False
        
    def print_stats(self):
        """Печать статистики"""
        print("\n📊 Статистика портов:")
        for name, data in self.stats.items():
            if data["packets"] > 0:
                print(f"  {name}: {data['packets']} пакетов, {data['bytes']} байт")
            else:
                print(f"  {name}: нет данных")

def send_activation_commands():
    """Отправка команд активации ESP32"""
    print("📤 Отправка команд активации...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # Расширенный набор команд для полной активации
        commands = [
            # Базовые команды
            ("PING", "Ping тест"),
            ("STATUS", "Запрос статуса"),
            ("0.0 0.0", "Стоп моторов"),
            
            # LIDAR команды (разные варианты)
            (bytes([0xA5, 0x20]), "LIDAR: сканирование"),
            (bytes([0xA5, 0x21]), "LIDAR: старт"),
            (bytes([0xA5, 0x25]), "LIDAR: непрерывное сканирование"),
            (bytes([0xA5, 0x40]), "LIDAR: информация"),
            
            # Команды датчиков
            ("SENSORS_ON", "Включение датчиков"),
            ("IMU_START", "Старт IMU"),
            ("1.0 1.0", "Тест моторов"),
        ]
        
        for cmd, desc in commands:
            try:
                if isinstance(cmd, str):
                    sock.sendto(cmd.encode('utf-8'), (ESP32_IP, CMD_PORT))
                else:
                    sock.sendto(cmd, (ESP32_IP, CMD_PORT))
                    
                print(f"  ✅ {desc}")
                time.sleep(0.5)  # Пауза между командами
                
            except Exception as e:
                print(f"  ❌ Ошибка {desc}: {e}")
                
    except Exception as e:
        print(f"❌ Общая ошибка активации: {e}")
    finally:
        sock.close()

def main():
    print("🚀 ESP32 Активатор + Мониторинг")
    print("=" * 50)
    
    # Запускаем мониторинг
    monitor = MultiPortMonitor()
    threads = monitor.start_monitoring()
    
    print("\n⏳ Запуск через 2 секунды...")
    time.sleep(2)
    
    # Отправляем команды активации
    send_activation_commands()
    
    print(f"\n📡 Мониторинг активен. Ждем данных...")
    print("Press Ctrl+C to stop")
    
    try:
        # Периодически отправляем команды и показываем статистику
        for cycle in range(10):  # 10 циклов по 5 секунд = 50 секунд
            time.sleep(5)
            
            print(f"\n🔄 Цикл {cycle + 1}/10:")
            monitor.print_stats()
            
            # Повторно отправляем активирующие команды
            if cycle % 3 == 2:  # Каждые 15 секунд
                print("🔄 Повторная активация...")
                send_activation_commands()
                
    except KeyboardInterrupt:
        print("\n⏹️ Остановка...")
    finally:
        monitor.stop_monitoring()
        time.sleep(1)  # Даем время остановиться
        
        print("\n📊 Финальная статистика:")
        monitor.print_stats()
        
        # Общий вывод
        total_packets = sum(data["packets"] for data in monitor.stats.values())
        if total_packets > 0:
            print(f"\n✅ Успех! Получено {total_packets} пакетов")
        else:
            print(f"\n❌ Данные не получены")
            print("💡 Рекомендации:")
            print("  1. Проверьте прошивку ESP32")
            print("  2. Перезагрузите ESP32")
            print("  3. Проверьте WiFi соединение")
            print("  4. Убедитесь что IP адрес корректный")

if __name__ == "__main__":
    main()