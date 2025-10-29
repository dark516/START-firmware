#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray
import math
import threading
import time
import queue
import select

# Бинарная структура датчиков
SENSOR_PACKET_FORMAT = '<LllhhB'
SENSOR_PACKET_SIZE = struct.calcsize(SENSOR_PACKET_FORMAT)

class ESP32Commander(Node):
    def __init__(self):
        super().__init__('esp32_commander')

        # --- Параметры подключения ---
        self.declare_parameter('esp32_ip', '192.168.125.224')
        self.declare_parameter('cmd_port', 3333)
        self.declare_parameter('sensor_port', 3335)
        self.declare_parameter('lidar_port', 3334)
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.cmd_port = int(self.get_parameter('cmd_port').value)
        self.sensor_port = int(self.get_parameter('sensor_port').value)
        self.lidar_port = int(self.get_parameter('lidar_port').value)

        # --- UDP сокеты ---
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sensor_sock.bind(('', self.sensor_port))
            self.sensor_sock.settimeout(0.1)
            self.get_logger().info(f'✅ UDP сокеты созданы')
            self.get_logger().info(f'   Команды → {self.esp32_ip}:{self.cmd_port}')
            self.get_logger().info(f'   Датчики ← порт {self.sensor_port}')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка создания UDP сокетов: {e}')
            rclpy.shutdown()
            return

        # --- ROS интерфейс ---
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub_left  = self.create_publisher(Int32, '/left_motor/encoder/delta', 10)
        self.pub_right = self.create_publisher(Int32, '/right_motor/encoder/delta', 10)
        self.pub_imu   = self.create_publisher(Imu, '/imu/bno055', 10)
        self.pub_lidar_raw = self.create_publisher(UInt8MultiArray, '/rplidar/raw', 10)

        # --- Таймер опроса UDP сенсоров ---
        self.create_timer(0.02, self.read_from_esp)  # 50 Гц

        # --- Предыдущие значения энкодеров ---
        self.prev_left = None
        self.prev_right = None
        self.max_delta_ticks = 100000

        # ===== 📊 СЧЁТЧИКИ ЛИДАРА =====
        self.lidar_packets_received = 0
        self.lidar_bytes_received = 0
        self.lidar_last_packet_time = None
        self.lidar_packets_per_second = 0
        self.lidar_bytes_per_second = 0
        
        self.lidar_packets_counter = 0
        self.lidar_bytes_counter = 0
        self.lidar_counter_start_time = time.time()
        
        # ✅ ДОБАВЛЕНО: диагностика проблем
        self.lidar_errors = 0
        self.lidar_timeouts = 0
        self.lidar_reconnects = 0
        self.lidar_max_chunk_size = 0
        
        # ✅ ДОБАВЛЕНО: очередь для асинхронной публикации
        self.lidar_queue = queue.Queue(maxsize=100)
        
        # ===== ТАЙМЕР СТАТИСТИКИ =====
        self.create_timer(2.0, self.print_lidar_stats)
        
        # ✅ ТАЙМЕР ПУБЛИКАЦИИ из очереди (отдельный поток)
        self.create_timer(0.001, self.publish_lidar_from_queue)  # 1000 Hz

        # --- Поток лидар TCP ---
        self.lidar_thread_stop = False
        self.lidar_thread = threading.Thread(target=self._lidar_tcp_reader_optimized, daemon=True)
        self.lidar_thread.start()

    def cmd_callback(self, msg: Twist):
        try:
            command = f"{msg.linear.x:.3f} {msg.angular.z:.3f}"
            self.cmd_sock.sendto(command.encode('utf-8'), (self.esp32_ip, self.cmd_port))
        except Exception as e:
            self.get_logger().warn(f"Ошибка UDP отправки: {e}")

    def read_from_esp(self):
        try:
            packets_read = 0
            max_packets_per_call = 5
            
            while packets_read < max_packets_per_call:
                try:
                    data, addr = self.sensor_sock.recvfrom(1024)
                    if data:
                        self._handle_binary_packet(data)
                        packets_read += 1
                except socket.timeout:
                    break
                except Exception as e:
                    self.get_logger().debug(f"Ошибка получения UDP пакета: {e}")
                    break
        except Exception as e:
            self.get_logger().error(f"Ошибка UDP чтения: {e}")

    def _handle_binary_packet(self, data: bytes):
        if len(data) != SENSOR_PACKET_SIZE:
            return

        try:
            timestamp, left_abs, right_abs, yaw_deg, accel_mmps2, packet_id = struct.unpack(SENSOR_PACKET_FORMAT, data)
            
            x_accel = accel_mmps2 / 1000.0
            
            if self.prev_left is None or self.prev_right is None:
                delta_left = 0
                delta_right = 0
            else:
                delta_left = left_abs - self.prev_left
                delta_right = right_abs - self.prev_right

            self.prev_left = left_abs
            self.prev_right = right_abs

            if abs(delta_left) > self.max_delta_ticks or abs(delta_right) > self.max_delta_ticks:
                self.get_logger().warn(f"⚠️ Отброшены аномальные тики: ΔL={delta_left}, ΔR={delta_right}")
                return

            self.pub_left.publish(Int32(data=delta_left))
            self.pub_right.publish(Int32(data=delta_right))

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.linear_acceleration.x = x_accel
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0

            yaw_rad = math.radians(yaw_deg)
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(yaw_rad / 2.0)
            imu_msg.orientation.w = math.cos(yaw_rad / 2.0)

            self.pub_imu.publish(imu_msg)

        except struct.error as e:
            self.get_logger().error(f"❌ Ошибка распаковки пакета: {e}")
            return

    # ✅ ОПТИМИЗИРОВАННОЕ ЧТЕНИЕ ЛИДАРА
    def _lidar_tcp_reader_optimized(self):
        """Оптимизированное чтение TCP с большим буфером и select()"""
        
        while not self.lidar_thread_stop and rclpy.ok():
            s = None
            try:
                self.get_logger().info(f"🔌 Подключение к LIDAR TCP {self.esp32_ip}:{self.lidar_port}...")
                
                # ✅ ОПТИМИЗАЦИЯ: настройки сокета
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)  # 256KB буфер
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)    # Отключить Nagle
                
                s.settimeout(5.0)  # Только для connect
                s.connect((self.esp32_ip, self.lidar_port))
                s.setblocking(False)  # ✅ Неблокирующий режим!
                
                self.get_logger().info("✅ LIDAR TCP подключен (оптимизированный режим)")
                self.lidar_reconnects += 1
                
                # Буфер для накопления данных
                accumulated_data = bytearray()
                last_flush_time = time.time()
                
                while rclpy.ok() and not self.lidar_thread_stop:
                    try:
                        # ✅ Используем select для проверки готовности данных
                        ready = select.select([s], [], [], 0.1)  # 100мс timeout
                        
                        if ready[0]:
                            # ✅ Читаем БОЛЬШИЙ буфер
                            chunk = s.recv(65536)  # 64KB вместо 8KB!
                            
                            if not chunk:
                                raise ConnectionError("Socket closed by remote")
                            
                            chunk_size = len(chunk)
                            
                            # Обновляем статистику
                            self.lidar_packets_received += 1
                            self.lidar_bytes_received += chunk_size
                            self.lidar_packets_counter += 1
                            self.lidar_bytes_counter += chunk_size
                            self.lidar_last_packet_time = time.time()
                            
                            if chunk_size > self.lidar_max_chunk_size:
                                self.lidar_max_chunk_size = chunk_size
                            
                            # ✅ Накапливаем данные
                            accumulated_data.extend(chunk)
                            
                            # ✅ Публикуем батчами каждые 10мс или при накоплении 8KB
                            current_time = time.time()
                            if (current_time - last_flush_time > 0.01) or (len(accumulated_data) > 8192):
                                if accumulated_data:
                                    # Добавляем в очередь для публикации
                                    try:
                                        self.lidar_queue.put_nowait(bytes(accumulated_data))
                                    except queue.Full:
                                        self.get_logger().warn("⚠️ Очередь лидара переполнена!")
                                    
                                    accumulated_data.clear()
                                    last_flush_time = current_time
                        else:
                            # Timeout - проверяем накопленные данные
                            self.lidar_timeouts += 1
                            if accumulated_data:
                                try:
                                    self.lidar_queue.put_nowait(bytes(accumulated_data))
                                except queue.Full:
                                    pass
                                accumulated_data.clear()
                            
                    except BlockingIOError:
                        # Нормально для неблокирующего сокета
                        continue
                    except socket.error as e:
                        if e.errno == 11:  # EAGAIN
                            continue
                        else:
                            raise
                        
            except Exception as e:
                self.get_logger().warn(f"⚠️ LIDAR TCP ошибка: {e}")
                self.lidar_errors += 1
            finally:
                try:
                    if s: 
                        s.close()
                except Exception:
                    pass
                if not self.lidar_thread_stop:
                    self.get_logger().info("🔄 Переподключение через 1 секунду...")
                    time.sleep(1.0)

    # ✅ АСИНХРОННАЯ ПУБЛИКАЦИЯ из очереди
    def publish_lidar_from_queue(self):
        """Публикует данные из очереди в ROS"""
        try:
            while not self.lidar_queue.empty():
                try:
                    data = self.lidar_queue.get_nowait()
                    msg = UInt8MultiArray()
                    msg.data = list(data)
                    self.pub_lidar_raw.publish(msg)
                except queue.Empty:
                    break
        except Exception as e:
            self.get_logger().error(f"Ошибка публикации лидара: {e}")

    # ✅ РАСШИРЕННАЯ СТАТИСТИКА
    def print_lidar_stats(self):
        """Выводит статистику лидара с диагностикой"""
        
        current_time = time.time()
        time_diff = current_time - self.lidar_counter_start_time
        
        if time_diff > 0:
            self.lidar_packets_per_second = self.lidar_packets_counter / time_diff
            self.lidar_bytes_per_second = self.lidar_bytes_counter / time_diff
            
            self.lidar_packets_counter = 0
            self.lidar_bytes_counter = 0
            self.lidar_counter_start_time = current_time
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("📊 СТАТИСТИКА ЛИДАРА:")
        self.get_logger().info(f"   📦 Всего пакетов:    {self.lidar_packets_received:,}")
        self.get_logger().info(f"   📂 Всего байт:       {self.lidar_bytes_received:,} ({self.lidar_bytes_received/1024:.1f} KB)")
        self.get_logger().info(f"   ⚡ Частота:          {self.lidar_packets_per_second:.1f} пакетов/сек")
        self.get_logger().info(f"   🔄 Скорость:         {self.lidar_bytes_per_second:.0f} байт/сек ({self.lidar_bytes_per_second/1024:.1f} KB/s)")
        
        if self.lidar_packets_received > 0:
            avg_packet_size = self.lidar_bytes_received / self.lidar_packets_received
            self.get_logger().info(f"   📏 Средний размер:   {avg_packet_size:.0f} байт")
        
        self.get_logger().info(f"   📈 Макс. чанк:       {self.lidar_max_chunk_size} байт")
        self.get_logger().info(f"   ⚠️  Ошибки:          {self.lidar_errors}")
        self.get_logger().info(f"   ⏱️  Таймауты:        {self.lidar_timeouts}")
        self.get_logger().info(f"   🔌 Переподключения:  {self.lidar_reconnects}")
        self.get_logger().info(f"   📬 Очередь:          {self.lidar_queue.qsize()}/{self.lidar_queue.maxsize}")
        
        if self.lidar_last_packet_time:
            time_since_last = current_time - self.lidar_last_packet_time
            if time_since_last < 1.0:
                self.get_logger().info(f"   ✅ Статус:           АКТИВЕН ({time_since_last:.2f}с)")
            elif time_since_last < 5.0:
                self.get_logger().info(f"   ⚠️  Статус:           ЗАДЕРЖКА ({time_since_last:.1f}с)")
            else:
                self.get_logger().info(f"   ❌ Статус:           ПОТЕРЯ СВЯЗИ ({time_since_last:.0f}с)")
        else:
            self.get_logger().info(f"   ❌ Статус:           НЕТ ДАННЫХ")
        
        # ✅ ПРЕДУПРЕЖДЕНИЕ о проблемах
        if self.lidar_packets_per_second < 50:
            self.get_logger().warn(f"⚠️ НИЗКАЯ ЧАСТОТА ЛИДАРА: {self.lidar_packets_per_second:.1f} пакетов/сек!")
        
        self.get_logger().info("=" * 60)

    def destroy_node(self):
        try:
            self.lidar_thread_stop = True
        except Exception:
            pass
        try:
            self.cmd_sock.close()
            self.sensor_sock.close()
        except Exception:
            pass
        
        self.get_logger().info("\n📊 ФИНАЛЬНАЯ СТАТИСТИКА:")
        self.get_logger().info(f"   Пакетов: {self.lidar_packets_received}, Байт: {self.lidar_bytes_received/1024:.1f} KB")
        self.get_logger().info(f"   Ошибок: {self.lidar_errors}, Переподключений: {self.lidar_reconnects}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Commander()
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