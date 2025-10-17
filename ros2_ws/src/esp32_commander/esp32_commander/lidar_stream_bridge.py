#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct
import math
import time
from threading import Thread, Lock

class LidarStreamBridge(Node):
    def __init__(self):
        super().__init__('lidar_stream_bridge')
        
        # Version
        self.version = "v1.0"
        self.get_logger().info(f"=== LIDAR Stream Bridge Node {self.version} ===")
        
        # Parameters
        self.declare_parameter('esp32_ip', '192.168.1.100')
        self.declare_parameter('esp32_port', 3333)
        self.declare_parameter('client_port', 3334)
        
        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.esp32_port = self.get_parameter('esp32_port').get_parameter_value().integer_value
        self.client_port = self.get_parameter('client_port').get_parameter_value().integer_value
        
        self.get_logger().info(f"ESP32 target: {self.esp32_ip}:{self.esp32_port}")
        self.get_logger().info(f"Client port: {self.client_port}")
        
        # Publisher
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        # UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(0.01)  # 10ms timeout for non-blocking receive
        self.socket.bind(('', self.client_port))
        
        # Data buffer and parsing
        self.raw_buffer = bytearray()
        self.buffer_lock = Lock()
        
        # Statistics
        self.total_bytes_received = 0
        self.total_packets_received = 0
        self.last_stats_time = time.time()
        self.stats_interval = 5.0  # Print stats every 5 seconds
        
        # LIDAR scan data
        self.scan_data = {}  # angle -> (distance, quality)
        self.last_scan_publish = time.time()
        self.scan_publish_interval = 0.1  # Publish scans every 100ms
        
        # Threading
        self.running = True
        self.udp_thread = Thread(target=self.udp_receive_loop)
        self.process_thread = Thread(target=self.process_data_loop)
        
        # Register with ESP32
        self.register_with_esp32()
        
        # Start threads
        self.udp_thread.start()
        self.process_thread.start()
        
        self.get_logger().info("LIDAR Stream Bridge initialized and ready")
    
    def register_with_esp32(self):
        """Register with ESP32 to start receiving data"""
        try:
            reg_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            reg_socket.sendto(b"REGISTER", (self.esp32_ip, self.esp32_port))
            reg_socket.settimeout(2.0)
            
            response, addr = reg_socket.recvfrom(1024)
            if response == b"ACK":
                self.get_logger().info(f"Successfully registered with ESP32 at {addr}")
            else:
                self.get_logger().warn(f"Unexpected response from ESP32: {response}")
                
            reg_socket.close()
            
        except Exception as e:
            self.get_logger().error(f"Failed to register with ESP32: {e}")
    
    def udp_receive_loop(self):
        """Continuously receive UDP packets and buffer them"""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)
                
                with self.buffer_lock:
                    self.raw_buffer.extend(data)
                    self.total_bytes_received += len(data)
                    self.total_packets_received += 1
                    
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"UDP receive error: {e}")
    
    def process_data_loop(self):
        """Process buffered data and extract LIDAR measurements"""
        while self.running:
            with self.buffer_lock:
                if len(self.raw_buffer) < 5:
                    time.sleep(0.001)  # 1ms delay if not enough data
                    continue
                
                # Look for LIDAR data packet start (0xA5 0x5A)
                start_idx = -1
                for i in range(len(self.raw_buffer) - 1):
                    if self.raw_buffer[i] == 0xA5 and self.raw_buffer[i + 1] == 0x5A:
                        start_idx = i
                        break
                
                if start_idx == -1:
                    # No packet start found, clear old data
                    if len(self.raw_buffer) > 1000:
                        self.raw_buffer = self.raw_buffer[-100:]
                    time.sleep(0.001)
                    continue
                
                # Remove data before packet start
                if start_idx > 0:
                    self.raw_buffer = self.raw_buffer[start_idx:]
                
                # Check if we have enough data for a complete packet
                if len(self.raw_buffer) < 5:
                    time.sleep(0.001)
                    continue
                
                # Parse packet length
                packet_type = self.raw_buffer[2]
                sample_count = self.raw_buffer[3]
                
                if packet_type == 0x81:  # Scan data packet
                    expected_length = 5 + sample_count * 2  # Header + samples
                    
                    if len(self.raw_buffer) >= expected_length:
                        # Extract and process this packet
                        packet = self.raw_buffer[:expected_length]
                        self.raw_buffer = self.raw_buffer[expected_length:]
                        self.process_scan_packet(packet)
                    else:
                        time.sleep(0.001)
                        continue
                else:
                    # Unknown packet type, skip this byte
                    self.raw_buffer = self.raw_buffer[1:]
            
            # Print statistics periodically
            self.print_statistics()
            
            # Publish scan if enough time has passed
            self.publish_scan_if_ready()
            
            time.sleep(0.001)  # Small delay to prevent overwhelming CPU
    
    def process_scan_packet(self, packet):
        """Process a single LIDAR scan data packet"""
        if len(packet) < 5:
            return
        
        sample_count = packet[3]
        start_angle_raw = struct.unpack('<H', packet[4:6])[0] if len(packet) >= 6 else 0
        start_angle = (start_angle_raw >> 1) / 64.0  # Convert to degrees
        
        # Process measurement samples
        for i in range(sample_count):
            sample_offset = 6 + i * 2
            if sample_offset + 1 < len(packet):
                sample_data = struct.unpack('<H', packet[sample_offset:sample_offset + 2])[0]
                
                distance = (sample_data & 0x3FFF) / 4.0  # Distance in mm, convert to meters
                quality = (sample_data >> 14) & 0x03
                
                # Calculate angle for this sample
                angle_per_sample = 1.0 / sample_count if sample_count > 0 else 0
                angle = start_angle + i * angle_per_sample
                angle_rad = math.radians(angle)
                
                # Store measurement
                if distance > 0.1 and quality > 0:  # Filter out invalid measurements
                    self.scan_data[angle_rad] = (distance / 1000.0, quality)  # Convert mm to meters
    
    def publish_scan_if_ready(self):
        """Publish LaserScan message if enough time has passed"""
        current_time = time.time()
        if current_time - self.last_scan_publish >= self.scan_publish_interval:
            if self.scan_data:
                self.publish_laser_scan()
                self.last_scan_publish = current_time
    
    def publish_laser_scan(self):
        """Create and publish LaserScan message from collected data"""
        if not self.scan_data:
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"
        
        # LIDAR specifications
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)  # 1 degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 0.1  # 100ms scan time
        scan.range_min = 0.15  # 15cm minimum range
        scan.range_max = 12.0  # 12m maximum range
        
        # Calculate number of measurements
        num_measurements = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * num_measurements
        scan.intensities = [0.0] * num_measurements
        
        # Fill in measurements
        for angle_rad, (distance, quality) in self.scan_data.items():
            # Convert angle to array index
            if scan.angle_min <= angle_rad <= scan.angle_max:
                index = int((angle_rad - scan.angle_min) / scan.angle_increment)
                if 0 <= index < num_measurements:
                    scan.ranges[index] = distance
                    scan.intensities[index] = float(quality * 64)  # Scale quality to intensity
        
        # Publish scan
        self.scan_publisher.publish(scan)
        
        # Clear scan data for next cycle
        self.scan_data.clear()
    
    def print_statistics(self):
        """Print reception statistics periodically"""
        current_time = time.time()
        if current_time - self.last_stats_time >= self.stats_interval:
            time_diff = current_time - self.last_stats_time
            bytes_per_sec = self.total_bytes_received / time_diff
            packets_per_sec = self.total_packets_received / time_diff
            
            self.get_logger().info(
                f"Stats: {self.total_bytes_received} bytes, {self.total_packets_received} packets, "
                f"{bytes_per_sec:.1f} B/s, {packets_per_sec:.1f} pkt/s"
            )
            
            self.total_bytes_received = 0
            self.total_packets_received = 0
            self.last_stats_time = current_time
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down LIDAR Stream Bridge...")
        self.running = False
        
        # Wait for threads to finish
        if hasattr(self, 'udp_thread'):
            self.udp_thread.join(timeout=1.0)
        if hasattr(self, 'process_thread'):
            self.process_thread.join(timeout=1.0)
        
        # Close socket
        if hasattr(self, 'socket'):
            self.socket.close()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LidarStreamBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()