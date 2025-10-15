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
        self.declare_parameter('host', '10.115.122.247')
        self.declare_parameter('port', 3334)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_min', 0.0)
        self.declare_parameter('angle_max', 6.28318)  # 2*pi
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('scan_frequency', 10.0)  # Гц
        
        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        
        # TCP подключение к ESP32
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            self.get_logger().info(f'✅ Подключено к ESP32 LIDAR bridge ({host}:{port})')
        except Exception as e:
            self.get_logger().error(f'❌ Не удалось подключиться: {e}')
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
            
            # STOP (если работал)
            self.send_command([0xA5, 0x25])
            time.sleep(0.1)
            
            # RESET
            self.send_command([0xA5, 0x40])
            time.sleep(2.0)
            
            # GET_INFO (опционально, для проверки)
            # self.send_command([0xA5, 0x50])
            # time.sleep(0.5)
            
            # SCAN (стандартное сканирование)
            self.send_command([0xA5, 0x20])
            time.sleep(0.5)
            
            self.get_logger().info('✅ Лидар запущен в режиме SCAN')
        except Exception as e:
            self.get_logger().error(f'Ошибка инициализации лидара: {e}')
    
    def send_command(self, cmd_bytes):
        """Отправка команды лидару"""
        try:
            self.sock.send(bytes(cmd_bytes))
        except Exception as e:
            self.get_logger().error(f'Ошибка отправки команды: {e}')
    
    def receive_loop(self):
        """Главный цикл приёма данных"""
        buffer = b''
        
        while self.running and rclpy.ok():
            try:
                # Получаем данные
                data = self.sock.recv(4096)
                if not data:
                    self.get_logger().warn('Соединение закрыто')
                    break
                
                buffer += data
                
                # Парсим буфер
                buffer = self.parse_buffer(buffer)
                
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Ошибка приёма: {e}')
                time.sleep(0.1)
    
    def parse_buffer(self, buffer):
        """Парсинг протокола RPLidar"""
        while len(buffer) >= 5:
            # Ищем начало пакета измерения
            # Формат: [START_FLAG1 | START_FLAG2 | (6bit_Quality | 2bit_flags) | angle_low | angle_high | distance_low | distance_high ]
            # START_FLAG1 = 0xXX (зависит от S bit)
            # START_FLAG2 = 0xXX
            
            # Упрощенный парсинг для RPLidar A1/C1 (стандартный режим)
            # Байт 0: [S | !S | C | angle[14]]
            # S = 1 если начало нового скана
            # C = 1 если проверка failed (качество плохое)
            
            byte0 = buffer[0]
            
            # Проверка на валидность первого байта
            start_bit = (byte0 >> 0) & 0x01
            not_start_bit = (byte0 >> 1) & 0x01
            
            # Валидация: start_bit должен быть инверсией not_start_bit
            if start_bit == not_start_bit:
                # Невалидный пакет, сдвигаем буфер
                buffer = buffer[1:]
                continue
            
            # Проверяем что достаточно данных для пакета
            if len(buffer) < 5:
                break
            
            # Парсим пакет (5 байт)
            quality = (byte0 >> 2) & 0x3F  # 6 бит качества (для некоторых моделей)
            check_bit = (byte0 >> 1) & 0x01
            new_scan = start_bit
            
            # Угол (15 бит, 1/64 градуса)
            angle_q6 = buffer[1] | ((buffer[2] << 8))
            angle_deg = (angle_q6 >> 1) / 64.0
            
            # Расстояние (14 бит, 1/4 мм)
            distance_q2 = buffer[3] | ((buffer[4] << 8))
            distance_mm = (distance_q2 >> 2) / 4.0
            distance_m = distance_mm / 1000.0
            
            # Обработка данных
            if new_scan:
                # Публикуем накопленный скан
                self.publish_scan()
            
            # Добавляем точку в текущий скан
            if distance_m > 0.01:  # Фильтр шума
                with self.scan_lock:
                    # Округляем угол до целого градуса
                    angle_int = int(angle_deg) % 360
                    self.scan_data[angle_int] = (distance_m, quality)
            
            # Убираем обработанный пакет
            buffer = buffer[5:]
        
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
            self.sock.close()
        except:
            pass
        self.get_logger().info('🔴 LIDAR Bridge остановлен')
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