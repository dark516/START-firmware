#!/usr/bin/env python3
"""
Simple demo script to visualize robot position data
Usage: python3 robot_position_demo.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
import math
import time

class RobotPositionDemo(Node):
    def __init__(self):
        super().__init__('robot_position_demo')
        
        # Subscribe to robot pose
        self.pose_sub = self.create_subscription(
            Pose2D, '/robot_pose2d', self.pose_callback, 10)
        
        # Subscribe to raw sensor data for comparison
        self.left_sub = self.create_subscription(
            Int32, '/encoder_left', self.left_encoder_callback, 10)
        self.right_sub = self.create_subscription(
            Int32, '/encoder_right', self.right_encoder_callback, 10)
        self.gyro_sub = self.create_subscription(
            Int32, '/gyro_yaw', self.gyro_callback, 10)
        
        # Store data
        self.current_pose = None
        self.left_ticks = 0
        self.right_ticks = 0
        self.gyro_yaw = 0
        
        # Position history for simple trail
        self.position_history = []
        self.max_history = 20
        
        # Timer to print status
        self.timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('🎯 Robot Position Demo started')
        self.get_logger().info('📊 Monitoring robot position and sensor data...')

    def pose_callback(self, msg):
        self.current_pose = msg
        
        # Add to history
        self.position_history.append((msg.x, msg.y))
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def gyro_callback(self, msg):
        self.gyro_yaw = msg.data

    def print_status(self):
        if self.current_pose is not None:
            # Convert angle to degrees
            yaw_deg = math.degrees(self.current_pose.theta)
            
            # Calculate distance from origin
            distance = math.sqrt(self.current_pose.x**2 + self.current_pose.y**2)
            
            print(f"\n{'='*60}")
            print(f"🤖 ROBOT STATUS")
            print(f"{'='*60}")
            print(f"📍 Position: X={self.current_pose.x:7.3f}m  Y={self.current_pose.y:7.3f}m")
            print(f"🧭 Heading:  {yaw_deg:7.1f}° (θ={self.current_pose.theta:.3f} rad)")
            print(f"📏 Distance from origin: {distance:.3f}m")
            print(f"🔧 Raw Data: L={self.left_ticks:6d} R={self.right_ticks:6d} Gyro={self.gyro_yaw:4d}°")
            
            # Show movement trail
            if len(self.position_history) > 1:
                print(f"🛤️  Recent path (last {len(self.position_history)} positions):")
                for i, (x, y) in enumerate(self.position_history[-5:]):  # Show last 5
                    marker = "🔴" if i == len(self.position_history[-5:]) - 1 else "⚫"
                    print(f"   {marker} ({x:6.3f}, {y:6.3f})")
            
            # Simple ASCII compass
            self.draw_compass(yaw_deg)
        else:
            print("\n⏳ Waiting for robot position data...")

    def draw_compass(self, yaw_deg):
        """Draw a simple ASCII compass showing robot orientation"""
        print(f"\n🧭 Robot Orientation (facing {yaw_deg:.0f}°):")
        
        # Normalize angle to 0-360
        yaw_deg = yaw_deg % 360
        
        # Simple 8-direction compass
        directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        
        # Find closest direction
        min_diff = 360
        closest_dir = "N"
        for i, angle in enumerate(angles):
            diff = abs(yaw_deg - angle)
            if diff > 180:
                diff = 360 - diff
            if diff < min_diff:
                min_diff = diff
                closest_dir = directions[i]
        
        # Draw compass
        compass = [
            "     N     ",
            "  NW + NE  ",
            " W   +   E ",
            "  SW + SE  ",
            "     S     "
        ]
        
        # Highlight current direction
        for line in compass:
            highlighted = line.replace(closest_dir, f"[{closest_dir}]")
            print(f"   {highlighted}")

def main():
    rclpy.init()
    node = RobotPositionDemo()
    
    try:
        print("\n" + "="*60)
        print("🎯 ROBOT POSITION DEMO")
        print("="*60)
        print("📊 Monitoring robot position, orientation, and sensor data")
        print("🎮 Drive your robot around to see the position tracking!")
        print("⛔ Press Ctrl+C to stop")
        print("="*60)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\n👋 Demo stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()