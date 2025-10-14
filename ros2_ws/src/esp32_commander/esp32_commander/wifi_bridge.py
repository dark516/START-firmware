#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
import math

class ESP32Commander(Node):
    def __init__(self):
        super().__init__('esp32_commander')

        # --- Параметры подключения ---
        self.declare_parameter('host', '10.115.122.247')
        self.declare_parameter('port', 3333)
        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)

        # --- TCP соединение ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            self.sock.setblocking(False)
            self.get_logger().info(f'✅ Подключено к ESP32 ({host}:{port})')
        except Exception as e:
            self.get_logger().error(f'❌ Не удалось подключиться: {e}')
            rclpy.shutdown()
            return

        # --- Подписка на cmd_vel ---
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        # --- Публикации ---
        self.pub_left = self.create_publisher(Int32, '/left_motor/encoder/delta', 10)
        self.pub_right = self.create_publisher(Int32, '/right_motor/encoder/delta', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/bno055', 10)

        # --- Таймер опроса ESP32 ---
        self.create_timer(0.05, self.read_from_esp)  # 20 Гц

        # --- Предыдущие значения энкодеров (абсолютные с ESP32) ---
        self.prev_left = None
        self.prev_right = None

        # --- RX-буфер для построчной обработки ---
        self._rx_buf = ""

        # --- Ограничение на максимум тиков за один апдейт (антишум) ---
        self.max_delta_ticks = 1000  # подстрой под свой робот

    # === Отправка команд ===
    def cmd_callback(self, msg: Twist):
        try:
            line = f"{msg.linear.x:.3f} {msg.angular.z:.3f}\n"
            self.sock.sendall(line.encode('utf-8'))
            self.get_logger().info(f"➡ Отправлено: {line.strip()}")
        except Exception as e:
            self.get_logger().warn(f"Ошибка при отправке: {e}")

    # === Приём данных от ESP32 ===
    def read_from_esp(self):
        try:
            while True:
                try:
                    data = self.sock.recv(1024)
                except BlockingIOError:
                    break  # данных больше нет

                if not data:
                    return

                self._rx_buf += data.decode('utf-8', errors='ignore')
                while '\n' in self._rx_buf:
                    line, self._rx_buf = self._rx_buf.split('\n', 1)
                    self._handle_line(line.strip())

        except Exception as e:
            self.get_logger().error(f"Ошибка при чтении: {e}")

    def _handle_line(self, line: str):
        if not line:
            return
        parts = line.split()
        if len(parts) < 2:
            self.get_logger().debug(f"Неполные данные от ESP32: '{line}'")
            return

        try:
            left_abs = int(parts[0])
            right_abs = int(parts[1])
            yaw_deg = int(parts[2]) if len(parts) >= 3 else None
            x_accel = float(parts[3]) if len(parts) >= 4 else None

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
                self.get_logger().warn(f"⚠️ Отброшены аномальные тики: ΔL={delta_left}, ΔR={delta_right} (raw line: '{line}')")
                return

            # Публикация дельт
            self.pub_left.publish(Int32(data=delta_left))
            self.pub_right.publish(Int32(data=delta_right))

            # Публикация IMU (если есть yaw)
            if yaw_deg is not None:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"

                # Ускорение (если пришло)
                if x_accel is not None:
                    imu_msg.linear_acceleration.x = x_accel
                    imu_msg.linear_acceleration.y = 0.0
                    imu_msg.linear_acceleration.z = 0.0

                # Кватернион из yaw
                yaw_rad = math.radians(float(yaw_deg))
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = math.sin(yaw_rad / 2.0)
                imu_msg.orientation.w = math.cos(yaw_rad / 2.0)

                self.pub_imu.publish(imu_msg)

            self.get_logger().info(f"📡 ΔL={delta_left}, ΔR={delta_right}, Yaw={yaw_deg if yaw_deg is not None else '-'}°, "
                                   f"X_Accel={x_accel:.3f} m/s²" if x_accel is not None else "")

        except ValueError:
            self.get_logger().debug(f"Получены нечисловые данные: '{line}'")
            return

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        self.get_logger().info("❌ Соединение закрыто")
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