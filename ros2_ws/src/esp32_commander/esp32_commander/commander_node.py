#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class ESP32Commander(Node):
    def __init__(self):
        super().__init__('esp32_commander')

        # Параметры
        self.declare_parameter('host', '192.168.125.241')
        self.declare_parameter('port', 3333)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        # Подключение по TCP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((host, port))
            self.get_logger().info(f'✅ Подключеffffно к ESP32 ({host}:{port})')
        except Exception as e:
            self.get_logger().error(f'❌ Не удалось подключиться: {e}')
            rclpy.shutdown()
            return

        # --- ROS: подписка на cmd_vel ---
        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        # --- ROS: публикация энкодеров и гироскопа ---
        self.pub_left = self.create_publisher(Int32, '/encoder_left', 10)
        self.pub_right = self.create_publisher(Int32, '/encoder_right', 10)
        self.pub_gyro_yaw = self.create_publisher(Int32, '/gyro_yaw', 10)

        # Таймер для приёма данных с ESP
        self.create_timer(0.05, self.read_from_esp)

    # === Обработка cmd_vel ===
    def cmd_callback(self, msg: Twist):
        """Отправляем линейную и угловую скорость на ESP32"""
        try:
            line = f"{msg.linear.x:.3f} {msg.angular.z:.3f}\n"
            self.sock.sendall(line.encode())
            self.get_logger().info(f"➡ Отправлено: {line.strip()}")
        except Exception as e:
            self.get_logger().warn(f"Ошибка при отправке: {e}")

    # === Приём данных с ESP32 ===
    def read_from_esp(self):
        """Читаем тики с ESP32 и публикуем в два топика"""
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
            if len(parts) >= 3:
                left = int(parts[0])
                right = int(parts[1])
                yaw = int(parts[2])

                self.pub_left.publish(Int32(data=left))
                self.pub_right.publish(Int32(data=right))
                self.pub_gyro_yaw.publish(Int32(data=yaw))

                self.get_logger().info(f"📡 Данные: L={left}, R={right}, Yaw={yaw}°")
            elif len(parts) >= 2:
                # Обратная совместимость с старым форматом
                left = int(parts[0])
                right = int(parts[1])

                self.pub_left.publish(Int32(data=left))
                self.pub_right.publish(Int32(data=right))

                self.get_logger().info(f"📡 Энкодеры: L={left}, R={right}")

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

