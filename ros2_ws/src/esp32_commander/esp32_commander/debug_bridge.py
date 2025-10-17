#!/usr/bin/env python3
"""
Простой отладочный мостик для проверки связи с ESP32
Сначала проверяем базовую UDP связь, потом добавляем лидар
"""
import rclpy
from rclpy.node import Node
import socket
import time
import threading
from sensor_msgs.msg import LaserScan
import math

class DebugBridge(Node):
    def __init__(self):
        super().__init__('debug_bridge')
        
        # Параметры
        self.declare_parameter('esp32_ip', '192.168.125.222')
        self.declare_parameter('debug_level', 'verbose')  # verbose, normal, quiet
        
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.debug_level = self.get_parameter('debug_level').value
        
        # UDP сокеты
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Порты
        self.cmd_port = 3333
        self.lidar_port = 3334  
        self.sensor_port = 3335
        
        # Статистика
        self.stats = {
            'commands_sent': 0,
            'lidar_packets': 0,
            'sensor_packets': 0,
            'lidar_bytes': 0,
            'sensor_bytes': 0,
            'start_time': time.time()
        }
        
        # Настройка сокетов
        try:
            self.lidar_sock.bind(('', self.lidar_port))
            self.lidar_sock.settimeout(0.1)
            
            self.sensor_sock.bind(('', self.sensor_port))
            self.sensor_sock.settimeout(0.1)
            
            self.get_logger().info("✅ Debug Bridge создан")
            self.get_logger().info(f"   ESP32: {self.esp32_ip}")
            self.get_logger().info(f"   Команды → :{self.cmd_port}")
            self.get_logger().info(f"   LIDAR ← :{self.lidar_port}")
            self.get_logger().info(f"   Датчики ← :{self.sensor_port}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка создания сокетов: {e}")
            return
        
        # Публикатор для LaserScan (если получим данные)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Запуск потоков
        self.running = True
        self.lidar_thread = threading.Thread(target=self.lidar_receiver, daemon=True)
        self.sensor_thread = threading.Thread(target=self.sensor_receiver, daemon=True)
        
        self.lidar_thread.start()
        self.sensor_thread.start()
        
        # Таймеры
        self.create_timer(1.0, self.send_test_commands)  # Команды каждую секунду
        self.create_timer(5.0, self.print_stats)         # Статистика каждые 5 сек
        
        # Сразу отправим команду регистрации
        self.register_with_esp32()
        
        self.get_logger().info("🔴 Debug Bridge запущен!")
    
    def register_with_esp32(self):
        """Регистрация у ESP32 для автоопределения IP"""
        commands = [
            b'\xFF\xFF\xFF\xFF',     # Пинг для регистрации
            b'PING',                 # Текстовый пинг
            b'0.0 0.0'               # Команда моторов
        ]
        
        self.get_logger().info("🔗 Регистрируемся у ESP32...")
        
        for i, cmd in enumerate(commands):
            try:
                self.cmd_sock.sendto(cmd, (self.esp32_ip, self.cmd_port))
                self.stats['commands_sent'] += 1
                if self.debug_level == 'verbose':
                    self.get_logger().info(f"  📤 Команда #{i+1}: {cmd} ({len(cmd)} байт)")
                time.sleep(0.2)
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка отправки команды: {e}")
    
    def send_test_commands(self):
        """Отправка тестовых команд каждую секунду"""
        timestamp = int(time.time())
        
        test_commands = [
            b'TEST',
            f"TIME_{timestamp}".encode(),
            b'\xA5\x20',  # LIDAR START SCAN
            b'0.1 0.0'    # Команда моторов
        ]
        
        for cmd in test_commands:
            try:
                self.cmd_sock.sendto(cmd, (self.esp32_ip, self.cmd_port))
                self.stats['commands_sent'] += 1
                if self.debug_level == 'verbose':
                    self.get_logger().debug(f"📤 Test: {cmd}")
            except Exception as e:
                if self.debug_level != 'quiet':
                    self.get_logger().debug(f"⚠️ Команда не отправлена: {e}")
    
    def lidar_receiver(self):
        """Поток приема LIDAR данных"""
        buffer = b''
        
        while self.running and rclpy.ok():
            try:
                data, addr = self.lidar_sock.recvfrom(4096)
                
                self.stats['lidar_packets'] += 1
                self.stats['lidar_bytes'] += len(data)
                
                # Первый пакет - детальный лог
                if self.stats['lidar_packets'] == 1:
                    self.get_logger().info(f"🎉 ПЕРВЫЙ LIDAR ПАКЕТ!")
                    self.get_logger().info(f"   От: {addr}")
                    self.get_logger().info(f"   Размер: {len(data)} байт")
                    self.get_logger().info(f"   Hex: {data[:20].hex()}")
                    self.get_logger().info(f"   Первые 10 байт: {list(data[:10])}")
                
                # Каждый 50й пакет
                elif self.stats['lidar_packets'] % 50 == 0:
                    self.get_logger().info(f"📦 LIDAR #{self.stats['lidar_packets']}: {len(data)} байт")
                
                # Простой парсинг для теста (создаем фиктивный LaserScan)
                if len(data) > 10:
                    self.create_test_scan(data)
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.debug_level == 'verbose':
                    self.get_logger().debug(f"LIDAR recv error: {e}")
    
    def sensor_receiver(self):
        """Поток приема данных датчиков"""
        while self.running and rclpy.ok():
            try:
                data, addr = self.sensor_sock.recvfrom(1024)
                
                self.stats['sensor_packets'] += 1
                self.stats['sensor_bytes'] += len(data)
                
                # Первый пакет датчиков
                if self.stats['sensor_packets'] == 1:
                    self.get_logger().info(f"🎉 ПЕРВЫЙ ПАКЕТ ДАТЧИКОВ!")
                    self.get_logger().info(f"   От: {addr}")
                    self.get_logger().info(f"   Размер: {len(data)} байт")
                    self.get_logger().info(f"   Hex: {data.hex()}")
                    
                    # Если 17 байт - это наша бинарная структура
                    if len(data) == 17:
                        self.get_logger().info("   ✅ Размер совпадает с SensorDataPacket!")
                
                elif self.stats['sensor_packets'] % 25 == 0:
                    self.get_logger().info(f"🔬 SENSOR #{self.stats['sensor_packets']}: {len(data)} байт")
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.debug_level == 'verbose':
                    self.get_logger().debug(f"Sensor recv error: {e}")
    
    def create_test_scan(self, raw_data):
        """Создаем тестовый LaserScan из полученных данных"""
        try:
            # Простейший LaserScan для теста
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = 'laser'
            
            scan.angle_min = 0.0
            scan.angle_max = 2.0 * math.pi
            scan.angle_increment = math.pi / 180.0  # 1 градус
            scan.time_increment = 0.0
            scan.scan_time = 0.1
            scan.range_min = 0.1
            scan.range_max = 15.0
            
            # Создаем фиктивные данные на основе сырых данных
            ranges = []
            for i in range(360):
                # Используем байты из raw_data для создания псевдо-расстояний
                if i < len(raw_data):
                    # Простая формула для тестирования
                    range_val = (raw_data[i] % 100) / 10.0 + 1.0  # 1.0 - 11.0 метров
                else:
                    range_val = 0.0
                ranges.append(range_val)
            
            scan.ranges = ranges
            scan.intensities = [50.0] * 360  # Фиктивная интенсивность
            
            self.scan_pub.publish(scan)
            
            # Лог только для первого скана
            if self.stats['lidar_packets'] <= 2:
                self.get_logger().info("✅ Опубликован тестовый /scan!")
                
        except Exception as e:
            self.get_logger().error(f"Ошибка создания scan: {e}")
    
    def print_stats(self):
        """Печать статистики"""
        uptime = time.time() - self.stats['start_time']
        
        self.get_logger().info("╔═══════ DEBUG STATS ════════╗")
        self.get_logger().info(f"║ Время работы: {uptime:.1f}s        ║")
        self.get_logger().info(f"║ Команд отправлено: {self.stats['commands_sent']}       ║")
        self.get_logger().info(f"║ LIDAR пакетов: {self.stats['lidar_packets']}          ║")
        self.get_logger().info(f"║ LIDAR байт: {self.stats['lidar_bytes']}            ║")
        self.get_logger().info(f"║ Sensor пакетов: {self.stats['sensor_packets']}         ║")  
        self.get_logger().info(f"║ Sensor байт: {self.stats['sensor_bytes']}           ║")
        self.get_logger().info("╚════════════════════════════╝")
        
        # Если нет данных - предупреждение
        if self.stats['lidar_packets'] == 0 and uptime > 10:
            self.get_logger().warn("⚠️ LIDAR данные не получены за 10+ секунд")
        if self.stats['sensor_packets'] == 0 and uptime > 10:
            self.get_logger().warn("⚠️ Sensor данные не получены за 10+ секунд")
    
    def destroy_node(self):
        """Очистка"""
        self.running = False
        try:
            self.cmd_sock.close()
            self.lidar_sock.close() 
            self.sensor_sock.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DebugBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()