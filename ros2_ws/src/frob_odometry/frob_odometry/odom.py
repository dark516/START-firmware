#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Int32
from tf2_ros import TransformBroadcaster

def normalize_angle(a):
    # В диапазон [-pi, pi)
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        # Параметры
        self.declare_parameters(
            namespace='',
            parameters=[
                ('meters_per_tick', 0.0),     # 0.0 => вычислить из radius и ticks_per_rev
                ('wheel_radius', 0.065),
                ('ticks_per_revolution', 1440),
                ('wheel_base', 0.3),
                ('update_rate', 10.0),
                ('publish_tf', False),
                ('odom_frame_id', 'odom'),
                ('base_frame_id', 'base_link'),
                ('encoder_min', -32768),
                ('encoder_max', 32767)
            ]
        )

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.ticks_per_revolution = int(self.get_parameter('ticks_per_revolution').value)
        self.meters_per_tick = float(self.get_parameter('meters_per_tick').value)
        if self.meters_per_tick <= 0.0:
            if self.ticks_per_revolution <= 0:
                raise ValueError('ticks_per_revolution must be > 0 to compute meters_per_tick')
            self.meters_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_revolution
            self.get_logger().info(f'meters_per_tick computed = {self.meters_per_tick:.9f} m/tick')

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self.base_frame_id = str(self.get_parameter('base_frame_id').value)
        self.encoder_min = int(self.get_parameter('encoder_min').value)
        self.encoder_max = int(self.get_parameter('encoder_max').value)

        # Состояние
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_ticks = 0
        self.right_ticks = 0

        self.last_time = self.get_clock().now()

        # Подписки
        self.create_subscription(Int32, '/left_motor/encoder/delta', self.left_callback, 10)
        self.create_subscription(Int32, '/right_motor/encoder/delta', self.right_callback, 10)

        # Таймер
        self.create_timer(max(1e-3, 1.0 / self.update_rate), self.update_odometry)

        # Паблишеры
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

    def handle_encoder_overflow(self, value: int) -> int:
        # Если действительно приходят абсолютные значения — это нужно.
        # Для delta-топика, как у вас, переполнения обычно уже учтены на стороне прошивки.
        if value > self.encoder_max:
            return value - (self.encoder_max - self.encoder_min)
        if value < self.encoder_min:
            return value + (self.encoder_max - self.encoder_min)
        return value

    def left_callback(self, msg: Int32):
        self.left_ticks += self.handle_encoder_overflow(msg.data)

    def right_callback(self, msg: Int32):
        self.right_ticks += self.handle_encoder_overflow(msg.data)

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / self.update_rate

        # Перевод тиков в метры
        left_dist = self.left_ticks * self.meters_per_tick
        right_dist = self.right_ticks * self.meters_per_tick

        # Сброс накопителей тиков
        self.left_ticks = 0
        self.right_ticks = 0

        # Инкременты
        delta_s = 0.5 * (left_dist + right_dist)
        delta_theta = (right_dist - left_dist) / self.wheel_base

        # Интегрирование (midpoint)
        if abs(delta_theta) < 1e-12:
            dx = delta_s * math.cos(self.theta)
            dy = delta_s * math.sin(self.theta)
        else:
            dx = delta_s * math.cos(self.theta + 0.5 * delta_theta)
            dy = delta_s * math.sin(self.theta + 0.5 * delta_theta)

        self.x += dx
        self.y += dy
        self.theta = normalize_angle(self.theta + delta_theta)

        # Публикация
        self.publish_odometry(dt, delta_s, delta_theta)
        if self.publish_tf:
            self.publish_transform()

        self.last_time = now
        self.get_logger().info(f"🤖 Robot Position: X={self.x:.3f} m, Y={self.y:.3f} m, Theta={math.degrees(self.theta):.1f}°")

    def publish_odometry(self, dt: float, delta_s: float, delta_theta: float):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        # Поза
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.quaternion_from_yaw(self.theta)

        # Базовые ковариации (настройте под свою механику)
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.02   # var(x)
        pose_cov[7] = 0.02   # var(y)
        pose_cov[35] = 0.05  # var(yaw)
        odom_msg.pose.covariance = pose_cov

        # Скорости в базе робота
        v = delta_s / dt
        w = delta_theta / dt

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = w

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.05  # var(vx)
        twist_cov[35] = 0.1  # var(wz)
        odom_msg.twist.covariance = twist_cov

        self.odom_pub.publish(odom_msg)

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = self.quaternion_from_yaw(self.theta)

        self.tf_broadcaster.sendTransform(transform)

    def quaternion_from_yaw(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()