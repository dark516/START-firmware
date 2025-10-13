#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class ESP32Commander(Node):
    def __init__(self):
        super().__init__('esp32_commander')
        self.get_logger().error(f'privet')
        # --- Параметры подключения ---
        self.declare_parameter('host', '192.168.106.201')
        self.declare_parameter('port', 3333)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        # --- TCP соединение ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            self.get_logger().info(f'✅ Подключено к ESP32 ({host}:{port})')
        except Exception as e:
            self.get_logger().error(f'❌ Не удалось подключиться: {e}')
            rclpy.shutdown()
            return

        # --- Подписка на cmd_vel ---
        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        # --- Публикации ---
        self.pub_left = self.create_publisher(Int32, '/encoder_left', 10)
        self.pub_right = self.create_publisher(Int32, '/encoder_right', 10)
        self.pub_yaw = self.create_publisher(Int32, '/gyro_yaw', 10)

        # --- Таймер опроса ESP32 ---
        self.create_timer(0.05, self.read_from_esp)

        # --- Предыдущие значения энкодеров ---
        self.prev_left = None
        self.prev_right = None

    # === Отправка команд ===
    def cmd_callback(self, msg: Twist):
        """Отправляем линейную и угловую скорость на ESP32"""
        try:
            line = f"{msg.linear.x:.3f} {msg.angular.z:.3f}\n"
            self.sock.sendall(line.encode())
            self.get_logger().info(f"➡ Отправлено: {line.strip()}")
        except Exception as e:
            self.get_logger().warn(f"Ошибка при отправке: {e}")

    # === Приём данных от ESP32 ===
    def read_from_esp(self):
        """Читаем тики и yaw с ESP32, публикуем дельты и yaw"""
        try:
            self.sock.setblocking(False)
            try:
                data = self.sock.recv(64)
            except BlockingIOError:
                return  # данных нет

            if not data:
                return

            line = data.decode().strip()
            if not line:
                return

            parts = line.split()
            if len(parts) >= 3:  # ожидаем: left right yaw
                try:
                    left = int(parts[0])
                    right = int(parts[1])
                    yaw = int(parts[2])
                    
                    # Вычисляем дельты энкодеров
                    if self.prev_left is not None and self.prev_right is not None:
                        delta_left = left - self.prev_left
                        delta_right = right - self.prev_right
                    else:
                        delta_left = 0
                        delta_right = 0

                    self.prev_left = left
                    self.prev_right = right

                    # Публикуем
                    self.pub_left.publish(Int32(data=delta_left))
                    self.pub_right.publish(Int32(data=delta_right))
                    self.pub_yaw.publish(Int32(data=yaw))

                    self.get_logger().info(f"📡 ΔL={delta_left}, ΔR={delta_right}, Yaw={yaw}")
                except ValueError:
                    # Игнорируем строки с нечисловыми данными
                    self.get_logger().debug(f"Получены нечисловые данные: {line}")
                    return
            else:
                # Логируем неполные данные для отладки
                self.get_logger().debug(f"Неполные данные от ESP32: {line}")

        except Exception as e:
            self.get_logger().error(f"Ошибка при чтении: {e}")

    def destroy_node(self):
        self.sock.close()
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

