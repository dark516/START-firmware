#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist
import math

class WifiBridge(Node):
    def __init__(self):
        super().__init__('esp32_wifi_bridge')
        
        self.declare_parameter('host', '192.168.1.100')
        self.declare_parameter('port', 3333)
        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        self.left_encoder_pub = self.create_publisher(Int32, '/left_motor/encoder/delta', 10)
        self.right_encoder_pub = self.create_publisher(Int32, '/right_motor/encoder/delta', 10)
        # TODO: Добавить паблишер для IMU, если нужно
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.get_logger().info(f"Connecting to ESP32 at {self.host}:{self.port}")
        self.sock = None
        self.connect_thread = threading.Thread(target=self.connection_loop)
        self.connect_thread.daemon = True
        self.connect_thread.start()

    def connection_loop(self):
        while rclpy.ok():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((self.host, self.port))
                    self.sock = s
                    self.get_logger().info(f"Successfully connected to ESP32 at {self.host}:{self.port}")
                    self.receive_loop(s)
            except (ConnectionRefusedError, OSError) as e:
                self.get_logger().warn(f"Connection failed: {e}. Retrying in 5 seconds...")
                self.sock = None
                # Используем таймер вместо прямого time.sleep, чтобы не блокировать ROS
                timer = self.create_timer(5.0, lambda: None)
                rclpy.spin_until_future_complete(self, timer.timer_future)
                self.destroy_timer(timer)


    def receive_loop(self, s):
        buffer = ""
        while rclpy.ok():
            try:
                data = s.recv(1024)
                if not data:
                    self.get_logger().error("Connection closed by ESP32.")
                    break
                
                buffer += data.decode('utf-8')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.parse_and_publish(line)

            except (socket.error, UnicodeDecodeError) as e:
                self.get_logger().error(f"Receive error: {e}. Disconnecting.")
                break
        self.sock = None

    def parse_and_publish(self, line):
        """
        Парсит строку от ESP32 формата "left_ticks right_ticks yaw x_accel"
        """
        line = line.strip()
        if not line:
            return
            
        try:
            parts = line.split()
            if len(parts) >= 2: # Нам нужны как минимум тики
                left_ticks = int(parts[0])
                right_ticks = int(parts[1])
                
                self.left_encoder_pub.publish(Int32(data=left_ticks))
                self.right_encoder_pub.publish(Int32(data=right_ticks))

                # Можно также распарсить и опубликовать IMU данные, если нужно
                # if len(parts) >= 4:
                #    yaw_deg = int(parts[2])
                #    accel_x = float(parts[3])
                #    # ... код для публикации Imu сообщения
            else:
                self.get_logger().warn(f"Received malformed data (not enough parts): '{line}'")

        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parsing data '{line}': {e}")

    def cmd_vel_callback(self, msg: Twist):
        """
        Формирует строку для ESP32 формата "linear_x angular_z"
        """
        v = msg.linear.x  # Линейная скорость
        w = msg.angular.z # Угловая скорость

        # ESP32 ожидает строку "float float\n"
        cmd_str = f"{v:.4f} {w:.4f}\n"
        
        if self.sock:
            try:
                self.sock.sendall(cmd_str.encode('utf-8'))
                # self.get_logger().info(f"Sent: {cmd_str.strip()}") # Раскомментируйте для отладки
            except socket.error as e:
                self.get_logger().error(f"Failed to send command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WifiBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()