#!/usr/bin/env python3
"""
Низкоуровневый анализ сетевого трафика ESP32
Проверяем UDP пакеты на сетевом уровне
"""
import socket
import time
import threading
import subprocess
import signal
import os
import struct

ESP32_IP = "192.168.125.222"
CMD_PORT = 3333
SENSOR_PORT = 3335
LIDAR_PORT = 3334

class NetworkAnalyzer:
    def __init__(self):
        self.tcpdump_process = None
        self.monitoring = False
        
    def start_tcpdump(self):
        """Запуск tcpdump для мониторинга UDP трафика"""
        try:
            # Фильтр для UDP трафика от/к ESP32
            filter_cmd = f"udp and host {ESP32_IP}"
            
            cmd = [
                "sudo", "tcpdump", "-i", "any", "-n", "-v", 
                "-X",  # Показать содержимое пакетов в hex
                filter_cmd
            ]
            
            print(f"🔍 Запуск tcpdump: {' '.join(cmd[2:])}")
            self.tcpdump_process = subprocess.Popen(
                cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            return True
            
        except Exception as e:
            print(f"❌ Ошибка запуска tcpdump: {e}")
            return False
    
    def read_tcpdump_output(self, duration=20):
        """Читаем вывод tcpdump"""
        if not self.tcpdump_process:
            return
            
        print("📡 Мониторинг сетевого трафика...")
        start_time = time.time()
        packet_count = 0
        
        try:
            while time.time() - start_time < duration:
                line = self.tcpdump_process.stdout.readline()
                if line:
                    packet_count += 1
                    print(f"📦 #{packet_count}: {line.strip()}")
                else:
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("⏹️ Остановка мониторинга...")
            
        finally:
            if self.tcpdump_process:
                self.tcpdump_process.terminate()
                self.tcpdump_process.wait()
                
        print(f"✅ Обнаружено {packet_count} UDP пакетов за {duration}с")
    
    def send_test_packets(self):
        """Отправляем тестовые пакеты для генерации трафика"""
        print("📤 Отправка тестовых UDP пакетов...")
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        try:
            test_data = [
                (CMD_PORT, b"0.0 0.0", "Motor command"),
                (CMD_PORT, bytes([0xA5, 0x20]), "LIDAR command"),  
                (CMD_PORT, b"PING", "Ping test"),
                (CMD_PORT, b"STATUS", "Status request"),
            ]
            
            for port, data, desc in test_data:
                try:
                    sent = sock.sendto(data, (ESP32_IP, port))
                    print(f"  📤 {desc}: {sent} байт на порт {port}")
                    time.sleep(0.5)
                except Exception as e:
                    print(f"  ❌ Ошибка {desc}: {e}")
                    
        finally:
            sock.close()
    
    def check_network_interface(self):
        """Проверяем сетевые интерфейсы"""
        print("🔧 Проверка сетевых интерфейсов...")
        
        try:
            # Проверяем интерфейсы
            result = subprocess.run(["ip", "addr", "show"], 
                                  capture_output=True, text=True)
            
            print("📋 Сетевые интерфейсы:")
            for line in result.stdout.split('\n'):
                if '192.168.125' in line or 'wl' in line or 'en' in line:
                    print(f"  {line.strip()}")
                    
            # Проверяем маршруты
            result = subprocess.run(["ip", "route", "show"], 
                                  capture_output=True, text=True)
            
            print("\n🛣️ Маршруты к 192.168.125.0/24:")
            for line in result.stdout.split('\n'):
                if '192.168.125' in line:
                    print(f"  {line.strip()}")
                    
        except Exception as e:
            print(f"❌ Ошибка проверки сети: {e}")
    
    def test_raw_socket_communication(self):
        """Тест с raw сокетами для максимально низкого уровня"""
        print("\n🔬 Тест низкоуровневой UDP связи...")
        
        # Создаем UDP сокет с максимальными правами
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.settimeout(2.0)
            
            # Привязываемся ко всем интерфейсам
            sock.bind(('0.0.0.0', 0))  # Любой свободный порт
            local_port = sock.getsockname()[1]
            print(f"🔌 Слушаем на порту {local_port}")
            
            # Отправляем эхо-запрос
            test_msg = b"LOW_LEVEL_PING"
            sock.sendto(test_msg, (ESP32_IP, CMD_PORT))
            print(f"📤 Отправлен low-level ping: {test_msg}")
            
            # Пробуем получить ответ
            try:
                data, addr = sock.recvfrom(1024)
                print(f"📥 Получен ответ от {addr}: {data}")
                return True
            except socket.timeout:
                print("⏰ Нет ответа на low-level ping")
                return False
                
        except Exception as e:
            print(f"❌ Ошибка raw socket: {e}")
            return False
        finally:
            sock.close()

def main():
    print("🕵️ Низкоуровневый анализ сети ESP32")
    print("=" * 50)
    
    analyzer = NetworkAnalyzer()
    
    # 1. Проверяем сетевые интерфейсы  
    analyzer.check_network_interface()
    
    # 2. Тест низкоуровневой связи
    ping_success = analyzer.test_raw_socket_communication()
    
    # 3. Запускаем tcpdump в фоне
    print(f"\n🔍 Запуск мониторинга сетевого трафика...")
    if analyzer.start_tcpdump():
        
        # Запускаем мониторинг в отдельном потоке
        monitor_thread = threading.Thread(
            target=analyzer.read_tcpdump_output, 
            args=(15,)
        )
        monitor_thread.start()
        
        # Даем время на запуск tcpdump
        time.sleep(2)
        
        # 4. Отправляем тестовые пакеты
        analyzer.send_test_packets()
        
        # 5. Ждем завершения мониторинга
        monitor_thread.join()
        
    else:
        print("❌ Не удалось запустить tcpdump")
        print("💡 Попробуйте: sudo tcpdump -i any -n udp and host 192.168.125.222")
    
    print("\n" + "=" * 50)
    print("📊 Результаты низкоуровневой диагностики:")
    print(f"🔌 Raw socket ping: {'✅ Успех' if ping_success else '❌ Неудача'}")
    print(f"🌐 Сетевые интерфейсы: ✅ Проверены")
    print(f"📡 Сетевой трафик: 📋 См. выше")
    
    if not ping_success:
        print("\n💡 Рекомендации:")
        print("1. ESP32 не отвечает на UDP - перезагрузите устройство")
        print("2. Проверьте прошивку и Serial Monitor")
        print("3. Возможно ESP32 использует другие порты")
        print("4. Проверьте брандмауэр: sudo ufw status")

if __name__ == "__main__":
    main()