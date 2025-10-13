#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler

class RobotLocalizationNode(Node):
    def __init__(self):
        super().__init__('robot_localization')
        
        # Robot physical parameters (you may need to adjust these)
        self.declare_parameter('wheel_separation', 0.3)  # distance between wheels in meters
        self.declare_parameter('wheel_radius', 0.065)    # wheel radius in meters  
        self.declare_parameter('ticks_per_revolution', 1440)  # encoder ticks per wheel revolution
        
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        
        # Calculate meters per tick
        wheel_circumference = 2 * math.pi * self.wheel_radius
        self.meters_per_tick = wheel_circumference / self.ticks_per_rev
        
        # Robot state
        self.x = 0.0  # X position in meters
        self.y = 0.0  # Y position in meters
        self.theta = 0.0  # Orientation in radians
        
        # Track time for velocity calculations
        self.prev_time = self.get_clock().now()
        
        # Current velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Subscribers
        self.sub_left = self.create_subscription(
            Int32, '/encoder_left', self.left_encoder_callback, 10)
        self.sub_right = self.create_subscription(
            Int32, '/encoder_right', self.right_encoder_callback, 10)
        self.sub_gyro = self.create_subscription(
            Int32, '/gyro_yaw', self.gyro_callback, 10)
        
        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_pose2d = self.create_publisher(Pose2D, '/robot_pose2d', 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Current encoder values
        self.left_ticks = 0
        self.right_ticks = 0
        self.current_yaw_deg = 0
        
        # Timer for publishing odometry
        self.timer = self.create_timer(0.1, self.publish_odometry)
        
        self.get_logger().info('🤖 Robot Localization Node started')
        self.get_logger().info(f'📏 Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'⚙️ Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'🔢 Meters per tick: {self.meters_per_tick:.6f}m')
    
    def left_encoder_callback(self, msg):
        self.left_ticks += msg.data  # msg.data is delta, so we accumulate
        self.get_logger().debug(f'⬅️ Left encoder delta: {msg.data}, total: {self.left_ticks}')
        self.calculate_odometry_from_deltas(msg.data, 0)
    
    def right_encoder_callback(self, msg):
        self.right_ticks += msg.data  # msg.data is delta, so we accumulate
        self.get_logger().debug(f'➡️ Right encoder delta: {msg.data}, total: {self.right_ticks}')
        self.calculate_odometry_from_deltas(0, msg.data)
    
    def gyro_callback(self, msg):
        self.current_yaw_deg = msg.data
        # Convert degrees to radians for internal calculations
        # Note: We use gyro data to correct for drift, but primarily rely on odometry
        # You can implement sensor fusion here if needed
    
    def calculate_odometry_from_deltas(self, delta_left, delta_right):
        # Skip if no movement
        if delta_left == 0 and delta_right == 0:
            return
            
        # Log the deltas we received
        self.get_logger().info(
            f'🔢 Raw Deltas - Left: {delta_left}, Right: {delta_right}'
        )
        
        # Convert to distances
        left_distance = delta_left * self.meters_per_tick
        right_distance = delta_right * self.meters_per_tick
        
        # Log distances
        self.get_logger().debug(
            f'📏 Distances - Left: {left_distance:.6f}m, Right: {right_distance:.6f}m'
        )
        
        # Calculate center distance (how far robot moved forward)
        center_distance = (left_distance + right_distance) / 2.0
        
        # Use GYRO for orientation instead of calculating from encoders!
        # Store old position for logging
        old_x, old_y, old_theta = self.x, self.y, self.theta
        
        # Get current orientation from gyro (convert degrees to radians)
        current_gyro_rad = math.radians(self.current_yaw_deg)
        
        # Use gyro heading directly
        new_theta = current_gyro_rad
        
        # For position calculation, use the average orientation during movement
        # This is between old orientation and new orientation
        avg_theta = (self.theta + new_theta) / 2.0
        
        self.get_logger().info(
            f'🎯 Movement - Center: {center_distance:.6f}m | Using Gyro: {self.current_yaw_deg}° '
            f'| L: {left_distance:.6f}m, R: {right_distance:.6f}m'
        )
        
        # Position changes in world coordinates using gyro orientation
        dx = center_distance * math.cos(avg_theta)
        dy = center_distance * math.sin(avg_theta)
        
        # Update position
        self.x += dx
        self.y += dy
        self.theta = new_theta
        
        # Log the position changes
        self.get_logger().info(
            f'📍 Position Change: ΔX={dx:.6f}m, ΔY={dy:.6f}m '
            f'| Old θ={math.degrees(old_theta):.1f}° → Gyro θ={self.current_yaw_deg}° (avg: {math.degrees(avg_theta):.1f}°)'
        )
        
        # Normalize angle to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Calculate velocities
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        if dt > 0.01:  # Only update velocities if enough time has passed (10ms)
            self.linear_vel = center_distance / dt
            self.angular_vel = angular_change / dt
            
            # Limit maximum velocities to reasonable values
            max_linear_vel = 2.0  # 2 m/s max
            max_angular_vel = 3.14  # pi rad/s max
            
            if abs(self.linear_vel) > max_linear_vel:
                self.get_logger().warn(f'Limiting linear velocity from {self.linear_vel:.3f} to {max_linear_vel}')
                self.linear_vel = max_linear_vel if self.linear_vel > 0 else -max_linear_vel
                
            if abs(self.angular_vel) > max_angular_vel:
                self.get_logger().warn(f'Limiting angular velocity from {self.angular_vel:.3f} to {max_angular_vel}')
                self.angular_vel = max_angular_vel if self.angular_vel > 0 else -max_angular_vel
        
        # Update previous time for next calculation
        self.prev_time = current_time
    
    def publish_odometry(self):
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set velocities
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel
        
        # Set covariance (you may want to tune these values)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # theta
        
        odom.twist.covariance[0] = 0.01   # vx
        odom.twist.covariance[35] = 0.01  # vtheta
        
        # Publish odometry
        self.pub_odom.publish(odom)
        
        # Publish simple 2D pose
        pose2d = Pose2D()
        pose2d.x = self.x
        pose2d.y = self.y
        pose2d.theta = self.theta
        self.pub_pose2d.publish(pose2d)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # Log current state
        yaw_deg = math.degrees(self.theta)
        gyro_diff = self.current_yaw_deg - yaw_deg
        
        # Log position only when there's significant movement or periodically
        if (abs(self.linear_vel) > 0.05 or abs(self.angular_vel) > 0.1 or 
            int(current_time.nanoseconds / 1e9) % 2 == 0):  # Every 2 seconds when stationary
            self.get_logger().info(
                f'🎯 Position: X={self.x:.4f}m, Y={self.y:.4f}m, θ={yaw_deg:.1f}° '
                f'| Vel: {self.linear_vel:.3f}m/s, {math.degrees(self.angular_vel):.1f}°/s '
                f'| Gyro: {self.current_yaw_deg}° (diff: {gyro_diff:.1f}°)'
            )

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
