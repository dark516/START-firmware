#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def quat_from_yaw(yaw):
    # yaw в радианах
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    class Q: pass
    q = Q()
    q.x = 0.0
    q.y = 0.0
    q.z = qz
    q.w = qw
    return q

class RobotLocalizationNode(Node):
    def __init__(self):
        super().__init__('robot_localization')

        # Параметры робота
        self.declare_parameter('wheel_separation', 0.3)
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('ticks_per_revolution', 1440)
        self.declare_parameter('publish_rate', 20.0)  # Гц

        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_revolution').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # Геометрия
        wheel_circumference = 2.0 * math.pi * self.wheel_radius
        self.meters_per_tick = wheel_circumference / self.ticks_per_rev

        # Состояние
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0          # рад
        self.linear_vel = 0.0     # м/с
        self.angular_vel = 0.0    # рад/с

        # Инкременты, накопленные с прошлого тика таймера
        self._acc_left = 0
        self._acc_right = 0

        # IMU
        self._have_imu = False
        self._imu_yaw_rad = 0.0

        # Время
        self.prev_time = self.get_clock().now()

        # Подписки — ИМЕНА ТОПИКОВ ИЗ ТВОЕГО wifi_bridge.py
        self.create_subscription(Int32, '/left_motor/encoder/delta', self.left_cb, 10)
        self.create_subscription(Int32, '/right_motor/encoder/delta', self.right_cb, 10)
        self.create_subscription(Imu, '/imu/bno055', self.imu_cb, 10)

        # Паблишеры
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_pose2d = self.create_publisher(Pose2D, '/robot_pose2d', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Таймер интеграции/публикации
        self.timer = self.create_timer(max(1e-3, 1.0 / self.publish_rate), self.integrate_and_publish)

        self.get_logger().info('🤖 Robot Localization Node started')
        self.get_logger().info(f'📏 wheel_separation={self.wheel_separation} m, wheel_radius={self.wheel_radius} m')
        self.get_logger().info(f'🔢 ticks_per_rev={self.ticks_per_rev}, meters_per_tick={self.meters_per_tick:.9f}')

    def left_cb(self, msg: Int32):
        self._acc_left += int(msg.data)

    def right_cb(self, msg: Int32):
        self._acc_right += int(msg.data)

    def imu_cb(self, msg: Imu):
        # yaw из кватерниона
        q = msg.orientation
        # безопасный yaw (rad)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._imu_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self._have_imu = True

    def integrate_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # Забрать накопленные дельты и обнулить
        dL_ticks = self._acc_left
        dR_ticks = self._acc_right
        self._acc_left = 0
        self._acc_right = 0

        # Интеграция одометрии
        old_theta = self.theta
        dL = dL_ticks * self.meters_per_tick
        dR = dR_ticks * self.meters_per_tick
        delta_s = 0.5 * (dL + dR)

        if self._have_imu:
            new_theta = self._imu_yaw_rad
            dtheta = normalize_angle(new_theta - old_theta)
            avg_theta = old_theta + 0.5 * dtheta
            self.theta = new_theta
        else:
            # Резерв по энкодерам
            dtheta = (dR - dL) / self.wheel_separation
            avg_theta = old_theta + 0.5 * dtheta
            self.theta = normalize_angle(old_theta + dtheta)

        self.x += delta_s * math.cos(avg_theta)
        self.y += delta_s * math.sin(avg_theta)

        # Скорости
        # Если энкодерных дельт нет — медленно «усаживаем» скорость к нулю
        if dL_ticks == 0 and dR_ticks == 0:
            self.linear_vel *= 0.8
            self.angular_vel *= 0.8
        else:
            self.linear_vel = delta_s / dt
            if self._have_imu:
                self.angular_vel = normalize_angle(self.theta - old_theta) / dt
            else:
                self.angular_vel = dtheta / dt

        # Паблиш /odom и TF
        self.publish_odom(now)
        self.publish_tf(now)

        self.prev_time = now

    def publish_odom(self, now):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quat_from_yaw(self.theta)
        odom.pose.pose.orientation.x = q.x
        odom.pose.pose.orientation.y = q.y
        odom.pose.pose.orientation.z = q.z
        odom.pose.pose.orientation.w = q.w

        # Ковариации (подстрой под свой робот)
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05

        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel

        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1

        self.pub_odom.publish(odom)

        pose2d = Pose2D()
        pose2d.x = self.x
        pose2d.y = self.y
        pose2d.theta = self.theta
        self.pub_pose2d.publish(pose2d)

    def publish_tf(self, now):
        q = quat_from_yaw(self.theta)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RobotLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()