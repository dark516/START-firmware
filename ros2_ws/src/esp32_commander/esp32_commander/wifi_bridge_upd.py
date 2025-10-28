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

# Бинарная структура датчиков (соответствует ESP32)
# <Lllhhb: timestamp(u32), left(i32), right(i32), yaw(i16), accel(i16), packet_id(u8)
SENSOR_PACKET_FORMAT = '<LllhhB'
SENSOR_PACKET_SIZE = struct.calcsize(SENSOR_PACKET_FORMAT)

class ESP32Commander(Node):
    def __init__(self):
        super().__init__('esp32_commander')

        # --- Параметры подключения ---
        self.declare_parameter('esp32_ip', '192.168.125.151')
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

        # --- Поток лидар TCP ---
        self.lidar_thread_stop = False
        self.lidar_thread = threading.Thread(target=self._lidar_tcp_reader, daemon=True)
        self.lidar_thread.start()

    # === Отправка команд ===
    def cmd_callback(self, msg: Twist):
        try:
            command = f"{msg.linear.x:.3f} {msg.angular.z:.3f}"
            self.cmd_sock.sendto(command.encode('utf-8'), (self.esp32_ip, self.cmd_port))
            self.get_logger().info(f"➡ UDP команда: {command}")
        except Exception as e:
            self.get_logger().warn(f"Ошибка UDP отправки: {e}")

    # === Приём данных от ESP32 через UDP ===
    def read_from_esp(self):
        try:
            packets_read = 0
            max_packets_per_call = 5
            
            while packets_read < max_packets_per_call:
                try:
                    data, addr = self.sensor_sock.recvfrom(1024)
                    if data:
                        # self.get_logger().info(f"📦 Получен UDP пакет размером {len(data)} байт от {addr}")
                        self._handle_binary_packet(data)
                        packets_read += 1
                except socket.timeout:
                    break
                except Exception as e:
                    self.get_logger().debug(f"Ошибка получения UDP пакета: {e}")
                    break
        except Exception as e:
            self.get_logger().error(f"Ошибка UDP чтения: {e}")

    # === ОБРАБОТКА БИНАРНОГО ПАКЕТА ===
    def _handle_binary_packet(self, data: bytes):
        """Обрабатывает бинарный пакет от ESP32"""
        
        # 1. Проверка размера
        if len(data) != SENSOR_PACKET_SIZE:
            # self.get_logger().error(f"❌ Неверный размер пакета: {len(data)}, ожидается {SENSOR_PACKET_SIZE}")
            # self.get_logger().info(f"   Сырые данные: {data.hex(' ')}")
            return

        try:
            # 2. Распаковываем бинарный пакет (int16 для yaw и accel)
            timestamp, left_abs, right_abs, yaw_deg, accel_mmps2, packet_id = struct.unpack(SENSOR_PACKET_FORMAT, data)
            
            # self.get_logger().info(f"📊 Пакет #{packet_id}:")
            # self.get_logger().info(f"   timestamp: {timestamp}")
            # self.get_logger().info(f"   left_abs: {left_abs}")
            # self.get_logger().info(f"   right_abs: {right_abs}")
            # self.get_logger().info(f"   yaw_deg: {yaw_deg}°")
            # self.get_logger().info(f"   accel_mmps2: {accel_mmps2}")
            
            # 3. Преобразования
            x_accel = accel_mmps2 / 1000.0  # мм/с² → м/с²
            
            # 4. Дельты энкодеров
            if self.prev_left is None or self.prev_right is None:
                delta_left = 0
                delta_right = 0
            else:
                delta_left = left_abs - self.prev_left
                delta_right = right_abs - self.prev_right

            self.prev_left = left_abs
            self.prev_right = right_abs

            # 5. Санити-чек
            if abs(delta_left) > self.max_delta_ticks or abs(delta_right) > self.max_delta_ticks:
                self.get_logger().warn(f"⚠️ Отброшены аномальные тики: ΔL={delta_left}, ΔR={delta_right}")
                return

            # 6. Публикация дельт
            self.pub_left.publish(Int32(data=delta_left))
            self.pub_right.publish(Int32(data=delta_right))

            # 7. Публикация IMU
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Ускорение
            imu_msg.linear_acceleration.x = x_accel
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0

            # Кватернион из yaw (-180..+180)
            yaw_rad = math.radians(yaw_deg)
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(yaw_rad / 2.0)
            imu_msg.orientation.w = math.cos(yaw_rad / 2.0)

            self.pub_imu.publish(imu_msg)

            # self.get_logger().info(f"✅ #{packet_id}: ΔL={delta_left}, ΔR={delta_right}, Yaw={yaw_deg}°, Acc={x_accel:.3f}m/s²")

        except struct.error as e:
            self.get_logger().error(f"❌ Ошибка распаковки пакета: {e}")
            return

    # === TCP чтение лидара ===
    def _lidar_tcp_reader(self):
        while not self.lidar_thread_stop and rclpy.ok():
            s = None
            try:
                self.get_logger().info(f"Подключение к LIDAR TCP {self.esp32_ip}:{self.lidar_port}...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5.0)
                s.connect((self.esp32_ip, self.lidar_port))
                s.settimeout(0.5)
                self.get_logger().info("LIDAR TCP подключен")

                while rclpy.ok() and not self.lidar_thread_stop:
                    try:
                        chunk = s.recv(8192)
                        if not chunk:
                            raise ConnectionError("socket closed")
                        msg = UInt8MultiArray()
                        msg.data = list(chunk)
                        self.pub_lidar_raw.publish(msg)
                    except socket.timeout:
                        continue
            except Exception as e:
                self.get_logger().warn(f"LIDAR TCP: {e}")
            finally:
                try:
                    if s: s.close()
                except Exception:
                    pass
                if not self.lidar_thread_stop:
                    time.sleep(1.0)

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
        self.get_logger().info("❌ UDP/TCP закрыты")
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