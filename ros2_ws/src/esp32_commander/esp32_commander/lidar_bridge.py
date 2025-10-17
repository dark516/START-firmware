#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
import math
from sensor_msgs.msg import LaserScan
from threading import Thread, Lock
import time

class RPLidarBridge(Node):
    """
    Прозрачный мост для RPLidar C1 через ESP32.
    Получает сырые данные по TCP, парсит протокол RPLidar и публикует LaserScan.
    """
    
    def __init__(self):
        super().__init__('rplidar_bridge')
        
        # Параметры подключения
        self.declare_parameter('esp32_ip', '192.168.125.222')
        self.declare_parameter('lidar_port', 3334)           # Порт для LIDAR данных
        self.declare_parameter('cmd_port', 3333)             # Порт для команд (общий с моторами)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_min', 0.0)
        self.declare_parameter('angle_max', 6.28318)  # 2*pi
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('scan_frequency', 10.0)  # Гц
        
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.lidar_port = int(self.get_parameter('lidar_port').value)
        self.cmd_port = int(self.get_parameter('cmd_port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        
        # UDP сокеты
        # Сокет для приема LIDAR данных
        self.lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Сокет для отправки команд LIDAR
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        try:
            self.lidar_sock.bind(('', self.lidar_port))
            self.lidar_sock.settimeout(0.1)  # Неблокирующий режим
            self.get_logger().info(f'✅ UDP LIDAR bridge создан')
            self.get_logger().info(f'   LIDAR данные ← порт {self.lidar_port}')
            self.get_logger().info(f'   LIDAR команды → {self.esp32_ip}:{self.cmd_port}')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка создания UDP сокетов: {e}')
            rclpy.shutdown()
            return
        
        # Публикатор LaserScan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Данные для накопления скана
        self.scan_data = {}  # {angle_deg: (distance_m, quality)}
        self.scan_lock = Lock()
        self.last_publish_time = time.time()
        self.scan_period = 1.0 / float(self.get_parameter('scan_frequency').value)
        
        # Запуск потока обработки данных
        self.running = True
        self.thread = Thread(target=self.receive_loop, daemon=True)
        self.thread.start()
        
        # Инициализация лидара
        self.init_lidar()
        
        self.get_logger().info('🔴 RPLidar Bridge запущен!')
    
    def init_lidar(self):
        """Инициализация и запуск лидара"""
        try:
            time.sleep(0.5)
            
            # ВАЖНО: Сначала регистрируемся у ESP32 отправив любую команду
            # ESP32 с автоопределением IP начнет отправлять данные только после первой команды
            self.get_logger().info('🔗 Регистрируемся у ESP32...')
            self.send_command([0xFF])  # Простая команда для регистрации IP
            time.sleep(0.5)
            
            # STOP (если работал)
            self.send_command([0xA5, 0x25])
            time.sleep(0.2)
            
            # RESET
            self.send_command([0xA5, 0x40])
            time.sleep(2.0)
            
            # START SCAN (стандартное сканирование)
            self.send_command([0xA5, 0x20])
            time.sleep(1.0)
            
            self.get_logger().info('✅ Лидар запущен в режиме SCAN')
        except Exception as e:
            self.get_logger().error(f'Ошибка инициализации лидара: {e}')
    
    def send_command(self, cmd_bytes):
        """Отправка команды лидару через UDP"""
        try:
            self.cmd_sock.sendto(bytes(cmd_bytes), (self.esp32_ip, self.cmd_port))
        except Exception as e:
            self.get_logger().error(f'Ошибка UDP отправки команды: {e}')
    
    def receive_loop(self):
        """Главный цикл приёма UDP данных"""
        buffer = b''
        packet_count = 0
        
        while self.running and rclpy.ok():
            try:
                # Получаем UDP пакет
                data, addr = self.lidar_sock.recvfrom(4096)
                if data:
                    packet_count += 1
                    if packet_count % 50 == 0:  # Логирование каждые 50 пакетов
                        self.get_logger().info(f'📦 LIDAR пакет #{packet_count}: {len(data)} байт от {addr}')
                    
                    # Накапливаем данные в буфер
                    buffer += data
                    
                    # Парсим буфер
                    buffer = self.parse_buffer(buffer)
                
            except socket.timeout:
                # Нормально - нет данных
                continue
            except Exception as e:
                self.get_logger().debug(f'Ошибка UDP приёма: {e}')
                time.sleep(0.01)
    
    def parse_buffer(self, buffer):
        """Парсинг RPLidar сырых данных (упрощенный)"""
        points_parsed = 0
        
        while len(buffer) >= 5:
            # Ищем начало RPLidar пакета
            # RPLidar C1 стандартный формат: 5 байт на точку
            
            try:
                byte0 = buffer[0]
                
                # Проверка на S и !S биты (RPLidar протокол)
                start_bit = (byte0 >> 0) & 0x01
                not_start_bit = (byte0 >> 1) & 0x01
                
                # Проверка валидности
                if start_bit == not_start_bit:
                    buffer = buffer[1:]  # Сдвиг на 1 байт
                    continue
                
                # Парсинг данных
                quality = (byte0 >> 2) & 0x3F
                new_scan_flag = start_bit
                
                # Угол (15 бит, 1/64 градуса)
                angle_q6 = buffer[1] | (buffer[2] << 8)
                angle_deg = (angle_q6 >> 1) / 64.0
                
                # Расстояние (16 бит, 1/4 мм)
                distance_q2 = buffer[3] | (buffer[4] << 8)
                distance_mm = distance_q2 / 4.0
                distance_m = distance_mm / 1000.0
                
                # Публикуем скан при новом скане или по таймеру
                current_time = time.time()
                if new_scan_flag or (current_time - self.last_publish_time) > self.scan_period:
                    self.publish_scan()
                
                # Добавляем точку в скан (фильтр минимального расстояния)
                if 0.05 < distance_m < 15.0 and quality > 5:  # Фильтр шума и качества
                    with self.scan_lock:
                        angle_int = int(angle_deg) % 360
                        self.scan_data[angle_int] = (distance_m, quality)
                
                points_parsed += 1
                buffer = buffer[5:]  # Убираем 5 байт
                
            except (IndexError, ValueError, struct.error):
                # Ошибка парсинга - сдвигаем на 1 байт
                buffer = buffer[1:]
        
        # Логирование каждые 100 точек
        if points_parsed > 0 and points_parsed % 100 == 0:
            self.get_logger().debug(f'📊 Обработано {points_parsed} точек LIDAR')
        
        return buffer
    
    def publish_scan(self):
        """Публикация LaserScan сообщения"""
        with self.scan_lock:
            if not self.scan_data:
                return
            
            # Создаём LaserScan
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id
            
            scan_msg.angle_min = 0.0
            scan_msg.angle_max = 2.0 * math.pi
            scan_msg.angle_increment = math.radians(1.0)  # 1 градус
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = self.scan_period
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max
            
            # Заполняем массивы
            ranges = [0.0] * 360
            intensities = [0.0] * 360
            
            for angle_deg, (distance, quality) in self.scan_data.items():
                idx = angle_deg % 360
                ranges[idx] = distance if self.range_min <= distance <= self.range_max else 0.0
                intensities[idx] = float(quality)
            
            scan_msg.ranges = ranges
            scan_msg.intensities = intensities
            
            # Публикуем
            self.scan_pub.publish(scan_msg)
            
            # Очищаем данные
            self.scan_data.clear()
            self.last_publish_time = time.time()
    
    def destroy_node(self):
        """Остановка и закрытие"""
        self.running = False
        try:
            # Останавливаем лидар
            self.send_command([0xA5, 0x25])  # STOP
            time.sleep(0.1)
            # Закрываем UDP сокеты
            self.lidar_sock.close()
            self.cmd_sock.close()
        except:
            pass
        self.get_logger().info('🔴 LIDAR UDP Bridge остановлен')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RPLidarBridge()
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