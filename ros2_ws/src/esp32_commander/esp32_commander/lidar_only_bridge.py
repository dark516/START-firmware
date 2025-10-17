#!/usr/bin/env python3
"""
Простой мост только для LIDAR - БЕЗ команд моторам!
Робот стоит неподвижно, только получаем и парсим данные лидара
"""
import rclpy
from rclpy.node import Node
import socket
import time
import threading
from sensor_msgs.msg import LaserScan
import math

class LidarOnlyBridge(Node):
    def __init__(self):
        super().__init__('lidar_only_bridge')
        
        # Параметры
        self.declare_parameter('esp32_ip', '192.168.125.222')
        self.esp32_ip = self.get_parameter('esp32_ip').value
        
        # Порты
        self.cmd_port = 3333
        self.lidar_port = 3334
        
        # UDP сокеты
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Статистика
        self.stats = {
            'registration_commands': 0,
            'lidar_packets': 0,
            'lidar_bytes': 0,
            'scan_published': 0,
            'start_time': time.time()
        }
        
        try:
            # Сокет для приема LIDAR данных
            self.lidar_sock.bind(('', self.lidar_port))
            self.lidar_sock.settimeout(0.1)
            
            self.get_logger().info("✅ LIDAR Only Bridge создан")
            self.get_logger().info(f"   ESP32: {self.esp32_ip}")
            self.get_logger().info(f"   LIDAR данные ← порт {self.lidar_port}")
            self.get_logger().info(f"   Команды → {self.esp32_ip}:{self.cmd_port}")
            self.get_logger().info("   🚫 Команды моторам НЕ отправляются - робот стоит!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка создания UDP сокетов: {e}")
            return
        
        # Публикатор LaserScan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Данные для сканирования
        self.scan_data = {}  # {angle_deg: distance_m}
        self.scan_lock = threading.Lock()
        
        # Поток приема данных
        self.running = True
        self.lidar_thread = threading.Thread(target=self.lidar_receiver, daemon=True)
        self.lidar_thread.start()
        
        # Таймеры
        self.create_timer(3.0, self.send_registration)    # Регистрация каждые 3 сек
        self.create_timer(10.0, self.print_stats)         # Статистика каждые 10 сек
        self.create_timer(0.1, self.publish_scan_timer)   # Публикация скана каждые 100мс
        
        # Сразу зарегистрируемся
        self.register_with_esp32()
        
        self.get_logger().info("🔴 LIDAR Only Bridge запущен!")
    
    def register_with_esp32(self):
        """Регистрация у ESP32"""
        registration_commands = [
            b'REGISTER_LIDAR',   # Текстовая регистрация
            b'\xFF',             # Простая регистрация  
            b'PING',             # Пинг
        ]
        
        self.get_logger().info("🔗 Регистрируемся у ESP32 (только для LIDAR)...")
        
        for i, cmd in enumerate(registration_commands):
            try:
                self.cmd_sock.sendto(cmd, (self.esp32_ip, self.cmd_port))
                self.stats['registration_commands'] += 1
                self.get_logger().info(f"  📤 Регистрация #{i+1}: {cmd}")
                time.sleep(0.3)
            except Exception as e:
                self.get_logger().error(f"❌ Ошибка регистрации: {e}")
    
    def send_registration(self):
        """Периодическая регистрация"""
        try:
            # Отправляем только команды регистрации, НЕ команды моторов!
            self.cmd_sock.sendto(b'KEEP_ALIVE', (self.esp32_ip, self.cmd_port))
            self.stats['registration_commands'] += 1
        except Exception as e:
            self.get_logger().debug(f"Ошибка keep-alive: {e}")
    
    def lidar_receiver(self):
        """Поток приема LIDAR данных"""
        buffer = b''
        
        while self.running and rclpy.ok():
            try:
                data, addr = self.lidar_sock.recvfrom(4096)
                
                self.stats['lidar_packets'] += 1
                self.stats['lidar_bytes'] += len(data)
                
                # Детали первого пакета
                if self.stats['lidar_packets'] == 1:
                    self.get_logger().info("🎉 ПЕРВЫЙ LIDAR ПАКЕТ ПОЛУЧЕН!")
                    self.get_logger().info(f"   От: {addr}")
                    self.get_logger().info(f"   Размер: {len(data)} байт")
                    self.get_logger().info(f"   Hex (первые 30): {data[:30].hex()}")
                
                # Логирование каждых 100 пакетов
                elif self.stats['lidar_packets'] % 100 == 0:
                    self.get_logger().info(f"📦 LIDAR пакет #{self.stats['lidar_packets']}: {len(data)} байт от {addr}")
                
                # Добавляем к буферу для парсинга
                buffer += data
                
                # Парсим буфер
                buffer = self.parse_lidar_buffer(buffer)
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().debug(f"Ошибка приема LIDAR: {e}")
    
    def parse_lidar_buffer(self, buffer):
        """Простой парсинг RPLidar данных"""
        points_parsed = 0
        
        while len(buffer) >= 5:
            try:
                # RPLidar протокол: 5 байт на точку
                byte0 = buffer[0]
                
                # Проверка S и !S битов
                start_bit = (byte0 >> 0) & 0x01
                not_start_bit = (byte0 >> 1) & 0x01
                
                if start_bit == not_start_bit:
                    buffer = buffer[1:]  # Невалидный, сдвигаем
                    continue
                
                # Парсим данные точки
                quality = (byte0 >> 2) & 0x3F
                new_scan = start_bit
                
                # Угол (15 бит, 1/64 градуса)
                angle_q6 = buffer[1] | (buffer[2] << 8)
                angle_deg = (angle_q6 >> 1) / 64.0
                
                # Расстояние (16 бит, 1/4 мм)
                distance_q2 = buffer[3] | (buffer[4] << 8)
                distance_mm = distance_q2 / 4.0
                distance_m = distance_mm / 1000.0
                
                # Добавляем точку в скан (с фильтрацией)
                if 0.05 < distance_m < 15.0 and quality > 3:
                    with self.scan_lock:
                        angle_int = int(angle_deg) % 360
                        self.scan_data[angle_int] = distance_m
                
                points_parsed += 1
                buffer = buffer[5:]  # Следующая точка
                
                # При новом скане - сразу публикуем (если есть данные)
                if new_scan and len(self.scan_data) > 10:
                    self.publish_scan_now()
                
            except (IndexError, ValueError):
                buffer = buffer[1:]  # Ошибка - сдвигаем на байт
        
        # Логирование парсинга
        if points_parsed > 0 and points_parsed % 200 == 0:
            self.get_logger().debug(f"🔍 Парсинг: обработано {points_parsed} точек")
        
        return buffer
    
    def publish_scan_timer(self):
        """Таймер публикации скана"""
        # Публикуем скан каждые 100мс если есть данные
        with self.scan_lock:
            if len(self.scan_data) > 20:  # Минимум 20 точек для публикации
                self.publish_scan_now()
    
    def publish_scan_now(self):
        """Публикация LaserScan сообщения"""
        with self.scan_lock:
            if not self.scan_data:
                return
            
            # Создаем LaserScan
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'laser'
            
            scan_msg.angle_min = 0.0
            scan_msg.angle_max = 2.0 * math.pi
            scan_msg.angle_increment = math.radians(1.0)  # 1 градус
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.1
            scan_msg.range_min = 0.05
            scan_msg.range_max = 15.0
            
            # Заполняем массивы (360 градусов)
            ranges = [0.0] * 360
            intensities = [0.0] * 360
            
            for angle_deg, distance in self.scan_data.items():
                idx = angle_deg % 360
                ranges[idx] = distance if scan_msg.range_min <= distance <= scan_msg.range_max else 0.0
                intensities[idx] = 50.0  # Фиксированная интенсивность
            
            scan_msg.ranges = ranges
            scan_msg.intensities = intensities
            
            # Публикуем
            self.scan_pub.publish(scan_msg)
            self.stats['scan_published'] += 1
            
            # Логирование публикации
            if self.stats['scan_published'] <= 3:
                self.get_logger().info(f"✅ Опубликован LaserScan #{self.stats['scan_published']} с {len(self.scan_data)} точками")
            elif self.stats['scan_published'] % 50 == 0:
                self.get_logger().info(f"📊 Опубликован LaserScan #{self.stats['scan_published']}")
            
            # Очищаем данные
            self.scan_data.clear()
    
    def print_stats(self):
        """Статистика работы"""
        uptime = time.time() - self.stats['start_time']
        
        self.get_logger().info("╔═══════════ LIDAR ONLY STATS ════════════╗")
        self.get_logger().info(f"║ Время работы: {uptime:.1f} сек")
        self.get_logger().info(f"║ Команд регистрации: {self.stats['registration_commands']}")
        self.get_logger().info(f"║ LIDAR пакетов: {self.stats['lidar_packets']}")
        self.get_logger().info(f"║ LIDAR байт: {self.stats['lidar_bytes']}")
        self.get_logger().info(f"║ LaserScan опубликовано: {self.stats['scan_published']}")
        
        # Скорость данных
        if uptime > 10:
            lidar_rate = self.stats['lidar_bytes'] / uptime
            scan_rate = self.stats['scan_published'] / uptime
            self.get_logger().info(f"║ Скорость LIDAR: {lidar_rate:.0f} байт/сек")
            self.get_logger().info(f"║ Частота LaserScan: {scan_rate:.1f} Гц")
        
        # Предупреждения
        if self.stats['lidar_packets'] == 0 and uptime > 15:
            self.get_logger().warn("║ ⚠️ LIDAR пакеты не получены!")
        elif self.stats['scan_published'] == 0 and uptime > 20:
            self.get_logger().warn("║ ⚠️ LaserScan не публикуется!")
        
        self.get_logger().info("╚══════════════════════════════════════════╝")
    
    def destroy_node(self):
        """Очистка при завершении"""
        self.running = False
        try:
            self.cmd_sock.close()
            self.lidar_sock.close()
        except:
            pass
        self.get_logger().info("🔴 LIDAR Only Bridge остановлен")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarOnlyBridge()
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