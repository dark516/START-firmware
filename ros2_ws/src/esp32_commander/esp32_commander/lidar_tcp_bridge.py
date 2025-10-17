#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math
import struct
import threading
import time
import binascii
import os
import numpy as np

class LidarTcpBridge(Node):
    def __init__(self):
        super().__init__('lidar_tcp_bridge')

        # Параметры подключения (как в ESP32Commander)
        self.declare_parameter('host', '192.168.1.72')
        self.declare_parameter('port', 3333)
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)

        # Raw data display options
        self.declare_parameter('show_raw_data', True)
        self.declare_parameter('raw_data_file', '')
        self.declare_parameter('raw_data_format', 'hex')  # hex, ascii, both
        self.declare_parameter('max_raw_bytes_per_display', 64)
        
        self.show_raw_data = self.get_parameter('show_raw_data').value
        self.raw_data_file = self.get_parameter('raw_data_file').value
        self.raw_data_format = self.get_parameter('raw_data_format').value
        self.max_raw_bytes = self.get_parameter('max_raw_bytes_per_display').value
        
        # Open raw data file if specified
        self.raw_file_handle = None
        if self.raw_data_file:
            try:
                self.raw_file_handle = open(self.raw_data_file, 'wb')
                self.get_logger().info(f"📝 Raw data logging to: {self.raw_data_file}")
            except Exception as e:
                self.get_logger().error(f"❌ Cannot open raw data file: {e}")

        self.get_logger().info("✅ LIDAR TCP Bridge для RAW данных создан")
        self.get_logger().info(f"   ESP32: {self.host}")
        self.get_logger().info(f"   LIDAR данные ← TCP порт {self.port}")
        self.get_logger().info(f"   📊 Показ сырых данных: {self.show_raw_data}")
        self.get_logger().info(f"   📄 Формат вывода: {self.raw_data_format}")
        self.get_logger().info(f"   📏 Максимум байт на дисплей: {self.max_raw_bytes}")

        # TCP соединение (точно как в ESP32Commander)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected = False
        self.connect_to_esp32()

        # Публикация LaserScan
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)

        # LIDAR parsing parameters
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_min', -math.pi)  # -180 degrees
        self.declare_parameter('angle_max', math.pi)   # +180 degrees
        self.declare_parameter('range_min', 0.15)      # 15cm minimum
        self.declare_parameter('range_max', 12.0)      # 12m maximum
        self.declare_parameter('scan_frequency', 10.0) # Hz
        
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_frequency = self.get_parameter('scan_frequency').value
        
        self.get_logger().info(f"📡 LaserScan параметры:")
        self.get_logger().info(f"   frame_id: {self.frame_id}")
        self.get_logger().info(f"   angle: {math.degrees(self.angle_min):.1f}° to {math.degrees(self.angle_max):.1f}°")
        self.get_logger().info(f"   range: {self.range_min}m to {self.range_max}m")
        self.get_logger().info(f"   frequency: {self.scan_frequency}Hz")

        # RX-буфер для бинарных данных LIDAR (не построчный как в ESP32Commander!)
        self._rx_buf = bytearray()
        
        # LIDAR data parsing
        self._current_scan = []
        self._scan_start_time = time.time()
        self._last_scan_publish = time.time()
        
        # Raw data tracking
        self.total_raw_bytes = 0
        self.raw_data_chunks = 0
        self.last_raw_display_time = time.time()

        # Статистика (обновленная для raw данных и LaserScan)
        self.stats = {
            'tcp_packets': 0,
            'raw_bytes': 0,
            'raw_chunks_displayed': 0,
            'file_bytes_written': 0,
            'laser_scans_published': 0,
            'total_scan_points': 0,
            'start_time': time.time()
        }

        # Таймер статистики
        self.create_timer(10.0, self.print_stats)

        # Поток для чтения данных TCP (как в рабочем ESP32Commander)
        self.read_thread = threading.Thread(target=self.tcp_reader_thread, daemon=True)
        self.read_thread.start()

        # Отправка команды START для активации LIDAR
        self.send_lidar_command("START")

    def connect_to_esp32(self):
        """Подключение к ESP32 TCP серверу"""
        try:
            self.sock.connect((self.host, self.port))
            self.sock.setblocking(False)
            self.connected = True
            self.get_logger().info(f'🔗 Подключено к ESP32 ({self.host}:{self.port})')
        except Exception as e:
            self.get_logger().error(f'❌ Не удалось подключиться: {e}')
            self.connected = False

    def send_lidar_command(self, command):
        """Отправка команды LIDAR (как в ESP32Commander)"""
        if not self.connected:
            return

        try:
            line = f"{command}\n"
            self.sock.sendall(line.encode('utf-8'))
            self.get_logger().info(f"📤 Команда LIDAR: {command}")
        except Exception as e:
            self.get_logger().warn(f"Ошибка при отправке команды: {e}")
            self.connected = False

    def tcp_reader_thread(self):
        """Поток для чтения TCP данных (адаптированный из ESP32Commander)"""
        while rclpy.ok():
            if not self.connected:
                time.sleep(1.0)
                continue

            try:
                data = self.sock.recv(4096)
                if not data:
                    self.get_logger().warn("TCP соединение разорвано")
                    self.connected = False
                    continue

                # Добавляем данные в буфер и отображаем сырые данные
                self._rx_buf.extend(data)
                self.stats['raw_bytes'] += len(data)
                self.stats['tcp_packets'] += 1
                self.total_raw_bytes += len(data)

                # Отображаем и записываем сырые данные
                self.process_raw_data(data)
                
                # Парсим LIDAR данные для LaserScan
                self.parse_lidar_data()
                
                # Ограничиваем размер буфера для предотвращения переполнения памяти
                if len(self._rx_buf) > 100000:  # 100KB max
                    self._rx_buf = self._rx_buf[-50000:]  # Keep last 50KB

            except socket.error:
                # Нет данных для чтения
                time.sleep(0.001)
                continue
            except Exception as e:
                self.get_logger().error(f"Ошибка чтения TCP: {e}")
                self.connected = False
                time.sleep(1.0)

    def process_raw_data(self, data):
        """Обработка и отображение сырых LIDAR данных"""
        current_time = time.time()
        
        # Запись в файл если указан
        if self.raw_file_handle:
            try:
                self.raw_file_handle.write(data)
                self.raw_file_handle.flush()
                self.stats['file_bytes_written'] += len(data)
            except Exception as e:
                self.get_logger().error(f"Ошибка записи в файл: {e}")
        
        # Отображение сырых данных (не чаще чем раз в секунду)
        if self.show_raw_data and (current_time - self.last_raw_display_time) >= 1.0:
            self.display_raw_data(data)
            self.last_raw_display_time = current_time
            self.stats['raw_chunks_displayed'] += 1
    
    def display_raw_data(self, data):
        """Отображение сырых данных в различных форматах"""
        display_bytes = data[:self.max_raw_bytes]
        
        self.get_logger().info("╔══════════════ RAW LIDAR DATA ═══════════════╗")
        self.get_logger().info(f"║ Размер пакета: {len(data)} байт")
        
        if self.raw_data_format in ['hex', 'both']:
            # Hex format
            hex_str = ' '.join([f'{b:02X}' for b in display_bytes])
            # Break long hex strings into lines
            max_hex_per_line = 24  # 24 hex values per line (48 chars)
            hex_lines = [hex_str[i:i+max_hex_per_line*3] for i in range(0, len(hex_str), max_hex_per_line*3)]
            
            self.get_logger().info("║ HEX:")
            for i, line in enumerate(hex_lines[:5]):  # Max 5 lines
                self.get_logger().info(f"║   {line}")
            if len(hex_lines) > 5:
                self.get_logger().info(f"║   ... ({len(hex_lines)-5} more lines)")
        
        if self.raw_data_format in ['ascii', 'both']:
            # ASCII format (printable chars only)
            ascii_chars = ''.join([chr(b) if 32 <= b <= 126 else '.' for b in display_bytes])
            # Break long ASCII strings into lines
            max_ascii_per_line = 64
            ascii_lines = [ascii_chars[i:i+max_ascii_per_line] for i in range(0, len(ascii_chars), max_ascii_per_line)]
            
            self.get_logger().info("║ ASCII:")
            for i, line in enumerate(ascii_lines[:5]):  # Max 5 lines
                self.get_logger().info(f"║   {line}")
            if len(ascii_lines) > 5:
                self.get_logger().info(f"║   ... ({len(ascii_lines)-5} more lines)")
        
        # Show pattern analysis
        self.analyze_data_patterns(display_bytes)
        
        self.get_logger().info("╚══════════════════════════════════════════════╝")
    
    def analyze_data_patterns(self, data):
        """Анализ паттернов в данных"""
        if len(data) < 4:
            return
        
        # Look for common LIDAR patterns
        patterns = {
            'A5 5A': [0xA5, 0x5A],  # RPLiDAR response header
            'A5 20': [0xA5, 0x20],  # Start scan response
            'A5 25': [0xA5, 0x25],  # Stop scan response
            '55 AA': [0x55, 0xAA],  # Possible sync pattern
        }
        
        found_patterns = []
        for pattern_name, pattern_bytes in patterns.items():
            for i in range(len(data) - len(pattern_bytes) + 1):
                if list(data[i:i+len(pattern_bytes)]) == pattern_bytes:
                    found_patterns.append(f"{pattern_name} at offset {i}")
        
        if found_patterns:
            self.get_logger().info(f"║ Паттерны: {', '.join(found_patterns[:3])}")
        
        # Byte frequency analysis
        byte_counts = {}
        for b in data:
            byte_counts[b] = byte_counts.get(b, 0) + 1
        
        most_common = sorted(byte_counts.items(), key=lambda x: x[1], reverse=True)[:3]
        if most_common:
            common_str = ', '.join([f'0x{b:02X}({count})' for b, count in most_common])
            self.get_logger().info(f"║ Частые байты: {common_str}")
    
    def parse_lidar_data(self):
        """Парсинг LIDAR данных из буфера в LaserScan"""
        while len(self._rx_buf) >= 5:  # Минимальный размер пакета
            # Ищем заголовок RPLiDAR (0xA5, 0x5A)
            sync_found = False
            sync_index = 0
            
            for i in range(len(self._rx_buf) - 1):
                if self._rx_buf[i] == 0xA5 and self._rx_buf[i + 1] == 0x5A:
                    sync_found = True
                    sync_index = i
                    break
            
            if not sync_found:
                # Удаляем первую половину буфера если синхронизация не найдена
                self._rx_buf = self._rx_buf[len(self._rx_buf)//2:]
                break
            
            # Удаляем данные до синхронизации
            if sync_index > 0:
                self._rx_buf = self._rx_buf[sync_index:]
            
            # Проверяем наличие достаточного количества данных для пакета
            if len(self._rx_buf) < 5:
                break
                
            # Читаем заголовок пакета
            packet_type = self._rx_buf[2]
            data_length = self._rx_buf[3]
            
            # Проверяем, есть ли полный пакет
            total_packet_length = 5 + data_length  # заголовок + данные + checksum
            if len(self._rx_buf) < total_packet_length:
                break  # Ждем больше данных
            
            # Извлекаем пакет
            packet = self._rx_buf[:total_packet_length]
            self._rx_buf = self._rx_buf[total_packet_length:]
            
            # Парсим пакет сканирования (0x81 = scan response)
            if packet_type == 0x81 and len(packet) >= 5:
                self.parse_scan_packet(packet[4:-1])  # Убираем заголовок и checksum
    
    def parse_scan_packet(self, scan_data):
        """Парсинг пакета данных сканирования"""
        # RPLiDAR scan data format: каждая точка = 5 байт
        # Байт 0: [S|NOT_S|C|0|0|0|Quality(5-0)]
        # Байт 1-2: Angle (little endian, 0.01 градуса)
        # Байт 3-4: Distance (little endian, 0.25 мм)
        
        points_count = len(scan_data) // 5
        current_time = time.time()
        
        for i in range(points_count):
            offset = i * 5
            if offset + 4 >= len(scan_data):
                break
                
            # Парсим точку
            quality_byte = scan_data[offset]
            quality = quality_byte & 0x3F  # 6 младших битов
            start_flag = (quality_byte & 0x80) != 0
            
            # Угол (0.01 градуса -> радианы)
            angle_raw = struct.unpack('<H', scan_data[offset+1:offset+3])[0]
            angle_deg = angle_raw * 0.01
            angle_rad = math.radians(angle_deg)
            
            # Расстояние (0.25 мм -> метры)
            distance_raw = struct.unpack('<H', scan_data[offset+3:offset+5])[0]
            distance_m = distance_raw * 0.00025
            
            # Фильтруем плохие измерения
            if quality < 10 or distance_m < self.range_min or distance_m > self.range_max:
                continue
                
            # Нормализуем угол в диапазон [-π, π]
            while angle_rad > math.pi:
                angle_rad -= 2 * math.pi
            while angle_rad < -math.pi:
                angle_rad += 2 * math.pi
            
            # Добавляем точку в текущий скан
            self._current_scan.append({
                'angle': angle_rad,
                'range': distance_m,
                'quality': quality
            })
            
            # Если это начало нового скана, публикуем предыдущий
            if start_flag and len(self._current_scan) > 100:  # Минимум 100 точек для публикации
                self.publish_laser_scan()
                self._current_scan = []
                self._scan_start_time = current_time
        
        # Публикуем скан по времени (если не было start_flag)
        if current_time - self._last_scan_publish > 1.0 / self.scan_frequency:
            if len(self._current_scan) > 50:  # Минимум 50 точек
                self.publish_laser_scan()
                self._current_scan = []
                self._scan_start_time = current_time
    
    def publish_laser_scan(self):
        """Публикация LaserScan сообщения"""
        if len(self._current_scan) == 0:
            return
            
        # Создаем LaserScan сообщение
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        # Вычисляем угловое разрешение
        angle_range = self.angle_max - self.angle_min
        angle_increment = angle_range / 360.0  # ~1 градус
        scan_msg.angle_increment = angle_increment
        
        # Создаем массив дальностей
        num_readings = int(angle_range / angle_increment)
        ranges = np.full(num_readings, float('inf'))
        intensities = np.zeros(num_readings)
        
        # Заполняем данные
        for point in self._current_scan:
            angle_index = int((point['angle'] - self.angle_min) / angle_increment)
            if 0 <= angle_index < num_readings:
                # Берем ближайшее расстояние если есть несколько точек в одном угле
                if ranges[angle_index] == float('inf') or point['range'] < ranges[angle_index]:
                    ranges[angle_index] = point['range']
                    intensities[angle_index] = point['quality']
        
        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = intensities.tolist()
        scan_msg.time_increment = 1.0 / (self.scan_frequency * num_readings)
        scan_msg.scan_time = 1.0 / self.scan_frequency
        
        # Публикуем
        self.pub_scan.publish(scan_msg)
        self._last_scan_publish = time.time()
        
        # Обновляем статистику
        self.stats['laser_scans_published'] += 1
        self.stats['total_scan_points'] += len(self._current_scan)
        
        # Логирование
        valid_points = sum(1 for r in ranges if r != float('inf'))
        self.get_logger().info(
            f"📡 LaserScan #{self.stats['laser_scans_published']}: {valid_points}/{num_readings} точек, "
            f"угол: {math.degrees(self.angle_min):.0f}° - {math.degrees(self.angle_max):.0f}°")

    def print_stats(self):
        """Печать статистики для RAW данных"""
        uptime = time.time() - self.stats['start_time']
        bytes_per_sec = self.stats['raw_bytes'] / uptime if uptime > 0 else 0
        packets_per_sec = self.stats['tcp_packets'] / uptime if uptime > 0 else 0
        chunks_per_sec = self.stats['raw_chunks_displayed'] / uptime if uptime > 0 else 0
        scans_per_sec = self.stats['laser_scans_published'] / uptime if uptime > 0 else 0
        avg_points_per_scan = self.stats['total_scan_points'] / max(1, self.stats['laser_scans_published'])

        self.get_logger().info("╔═════════════ RAW LIDAR STATS ════════════════╗")
        self.get_logger().info(f"║ Время работы: {uptime:.1f} сек")
        self.get_logger().info(f"║ TCP пакетов: {self.stats['tcp_packets']} ({packets_per_sec:.1f}/сек)")
        self.get_logger().info(f"║ Сырых байт: {self.stats['raw_bytes']} ({bytes_per_sec:.0f} байт/сек)")
        self.get_logger().info(f"║ Отображено чанков: {self.stats['raw_chunks_displayed']} ({chunks_per_sec:.2f}/сек)")
        self.get_logger().info(f"║ 📡 LaserScans: {self.stats['laser_scans_published']} ({scans_per_sec:.2f}/сек)")
        self.get_logger().info(f"║ 📊 Точек на скан: {avg_points_per_scan:.0f} среднем")
        
        if self.raw_file_handle:
            mb_written = self.stats['file_bytes_written'] / (1024 * 1024)
            self.get_logger().info(f"║ Записано в файл: {mb_written:.2f} MB")
        
        # Buffer status
        buffer_kb = len(self._rx_buf) / 1024
        self.get_logger().info(f"║ Размер буфера: {buffer_kb:.1f} KB")
        
        if not self.connected:
            self.get_logger().warn("║ ⚠️ TCP соединение разорвано!")
        elif self.stats['raw_bytes'] == 0:
            self.get_logger().warn("║ ⚠️ LIDAR данные не получены!")
        elif bytes_per_sec < 1000:
            self.get_logger().warn("║ ⚠️ Низкая скорость LIDAR!")
        else:
            self.get_logger().info("║ ✅ RAW LIDAR поток активен")
            
        self.get_logger().info("╚══════════════════════════════════════════════╝")

    def destroy_node(self):
        """Закрытие соединений и файлов"""
        try:
            if self.connected:
                self.send_lidar_command("STOP")
                time.sleep(0.1)
            self.sock.close()
        except Exception:
            pass
        
        # Close raw data file
        if self.raw_file_handle:
            try:
                self.raw_file_handle.close()
                self.get_logger().info(f"📝 Raw data file closed: {self.raw_data_file}")
            except Exception as e:
                self.get_logger().error(f"Error closing raw data file: {e}")
        
        self.get_logger().info("❌ TCP соединение закрыто")
        super().destroy_node()


# Класс RPLidarParser удален, так как мы работаем с сырыми данными


def main(args=None):
    rclpy.init(args=args)
    node = LidarTcpBridge()
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