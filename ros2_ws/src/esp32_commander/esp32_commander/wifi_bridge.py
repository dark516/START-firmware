#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
import math
import threading

# Бинарная структура датчиков (должна совпадать с ESP32)
SENSOR_PACKET_FORMAT = '<LLLHHB'  # Little-endian: timestamp, left_ticks, right_ticks, yaw, accel_x, packet_id
SENSOR_PACKET_SIZE = struct.calcsize(SENSOR_PACKET_FORMAT)

class ESP32Commander(Node):
    def __init__(self):
        super().__init__('esp32_commander')

        # --- Параметры подключения ---
        self.declare_parameter('esp32_ip', '192.168.125.222')  # IP ESP32
        self.declare_parameter('cmd_port', 3333)              # Порт для команд
        self.declare_parameter('sensor_port', 3335)           # Порт для данных датчиков
        
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.cmd_port = int(self.get_parameter('cmd_port').value)
        self.sensor_port = int(self.get_parameter('sensor_port').value)

        # --- UDP сокеты ---
        # Сокет для отправки команд на ESP32
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Сокет для приема данных датчиков
        self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sensor_sock.bind(('', self.sensor_port))
            self.sensor_sock.settimeout(0.1)  # Неблокирующий режим
            self.get_logger().info(f'✅ UDP сокеты созданы')
            self.get_logger().info(f'   Команды → {self.esp32_ip}:{self.cmd_port}')
            self.get_logger().info(f'   Датчики ← порт {self.sensor_port}')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка создания UDP сокетов: {e}')
            rclpy.shutdown()
            return

        # --- Подписка на cmd_vel ---
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # --- Публикации ---
        self.pub_left = self.create_publisher(Int32, '/left_motor/encoder/delta', 10)
        self.pub_right = self.create_publisher(Int32, '/right_motor/encoder/delta', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/bno055', 10)

        # --- Таймер опроса ESP32 ---
        self.create_timer(0.02, self.read_from_esp)  # 50 Гц (быстрее для UDP)

        # --- Предыдущие значения энкодеров (абсолютные с ESP32) ---
        self.prev_left = None
        self.prev_right = None

        # --- Ограничение на максимум тиков за один апдейт (антишум) ---
        self.max_delta_ticks = 1000  # подстрой под свой робот

    # === Отправка команд ===
    def cmd_callback(self, msg: Twist):
        try:
            # Формат: "linear_x angular_z"
            command = f"{msg.linear.x:.3f} {msg.angular.z:.3f}"
            self.cmd_sock.sendto(command.encode('utf-8'), (self.esp32_ip, self.cmd_port))
            self.get_logger().info(f"➡ UDP команда: {command}")
        except Exception as e:
            self.get_logger().warn(f"Ошибка UDP отправки: {e}")

    # === Приём данных от ESP32 через UDP ===
    def read_from_esp(self):
        try:
            # Пробуем получить несколько UDP пакетов за один вызов
            packets_read = 0
            max_packets_per_call = 5  # Ограничиваем количество пакетов
            
            while packets_read < max_packets_per_call:
                try:
                    data, addr = self.sensor_sock.recvfrom(1024)
                    if data:
                        # Обрабатываем бинарные данные
                        self._handle_binary_packet(data)
                        packets_read += 1
                except socket.timeout:
                    break  # Нет больше пакетов
                except Exception as e:
                    self.get_logger().debug(f"Ошибка получения UDP пакета: {e}")
                    break
        except Exception as e:
            self.get_logger().error(f"Ошибка UDP чтения: {e}")

    def _handle_binary_packet(self, data: bytes):
        """Обрабатывает бинарный пакет от ESP32"""
        if len(data) != SENSOR_PACKET_SIZE:
            self.get_logger().debug(f"Неверный размер пакета: {len(data)}, ожидается {SENSOR_PACKET_SIZE}")
            return

        try:
            # Распаковываем бинарный пакет
            timestamp, left_abs, right_abs, yaw_raw, accel_raw, packet_id = struct.unpack(SENSOR_PACKET_FORMAT, data)
            
            # Преобразуем данные
            yaw_deg = yaw_raw / 10.0      # Было умножено на 10
            x_accel = accel_raw / 1000.0  # Было в мм/с², переводим в м/с²
            
            # Дельты энкодеров из абсолютных значений
            if self.prev_left is None or self.prev_right is None:
                delta_left = 0
                delta_right = 0
            else:
                delta_left = left_abs - self.prev_left
                delta_right = right_abs - self.prev_right

            self.prev_left = left_abs
            self.prev_right = right_abs

            # Санити-чек
            if abs(delta_left) > self.max_delta_ticks or abs(delta_right) > self.max_delta_ticks:
                self.get_logger().warn(f"⚠️ Отброшены аномальные тики: ΔL={delta_left}, ΔR={delta_right} (packet #{packet_id})")
                return

            # Публикация дельт
            self.pub_left.publish(Int32(data=delta_left))
            self.pub_right.publish(Int32(data=delta_right))

            # Публикация IMU
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Ускорение
            imu_msg.linear_acceleration.x = x_accel
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0

            # Кватернион из yaw
            yaw_rad = math.radians(yaw_deg)
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(yaw_rad / 2.0)
            imu_msg.orientation.w = math.cos(yaw_rad / 2.0)

            self.pub_imu.publish(imu_msg)

            self.get_logger().info(f"📡 #{packet_id}: ΔL={delta_left}, ΔR={delta_right}, Yaw={yaw_deg:.1f}°, Acc={x_accel:.3f}m/s²")

        except struct.error as e:
            self.get_logger().debug(f"Ошибка распаковки пакета: {e}")
            return

    def destroy_node(self):
        try:
            self.cmd_sock.close()
            self.sensor_sock.close()
        except Exception:
            pass
        self.get_logger().info("❌ UDP сокеты закрыты")
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