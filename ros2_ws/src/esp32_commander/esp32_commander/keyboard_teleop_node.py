#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Create publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Store terminal settings for restoration
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('🎮 Keyboard Teleop Node started')
        self.get_logger().info('📝 Usage: Enter "linear_x angular_z" (e.g., "0.5 0.2")')
        self.get_logger().info('   - linear_x: forward/backward speed (m/s)')
        self.get_logger().info('   - angular_z: rotation speed (rad/s)')
        self.get_logger().info('   - Enter "stop" or "0 0" to stop the robot')
        self.get_logger().info('   - Enter "quit" or "exit" to quit')
        self.get_logger().info('---')
        
    def get_key(self):
        """Get a single keypress from stdin"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def get_input(self):
        """Get line input from user"""
        try:
            return input("Enter command (linear angular): ").strip()
        except EOFError:
            return "quit"
        except KeyboardInterrupt:
            return "quit"
    
    def parse_command(self, command):
        """Parse user command into linear and angular velocities"""
        if not command:
            return None, None
            
        command = command.lower()
        
        # Handle special commands
        if command in ['quit', 'exit', 'q']:
            return 'quit', 'quit'
        elif command in ['stop', 's']:
            return 0.0, 0.0
        
        # Parse numeric input
        try:
            parts = command.split()
            if len(parts) == 1:
                # If only one number, assume it's linear velocity
                linear = float(parts[0])
                angular = 0.0
            elif len(parts) == 2:
                # Two numbers: linear and angular
                linear = float(parts[0])
                angular = float(parts[1])
            else:
                self.get_logger().warn('❌ Invalid input format. Use: "linear angular" (e.g., "0.5 0.2")')
                return None, None
                
            return linear, angular
            
        except ValueError:
            self.get_logger().warn('❌ Invalid numbers. Please enter valid numbers.')
            return None, None
    
    def publish_velocity(self, linear_x, angular_z):
        """Publish velocity command to /cmd_vel"""
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(angular_z)
        
        self.publisher.publish(twist_msg)
        
        # Log the command
        if linear_x == 0.0 and angular_z == 0.0:
            self.get_logger().info('🛑 STOP command sent')
        else:
            direction = ""
            if linear_x > 0:
                direction += f"Forward {linear_x:.2f}m/s"
            elif linear_x < 0:
                direction += f"Backward {abs(linear_x):.2f}m/s"
            
            if angular_z > 0:
                if direction:
                    direction += ", "
                direction += f"Turn Left {angular_z:.2f}rad/s"
            elif angular_z < 0:
                if direction:
                    direction += ", "
                direction += f"Turn Right {abs(angular_z):.2f}rad/s"
                
            if not direction:
                direction = "No movement"
                
            self.get_logger().info(f'🚀 Command sent: {direction}')
    
    def run(self):
        """Main loop for getting user input and publishing commands"""
        try:
            while rclpy.ok():
                # Get user input
                command = self.get_input()
                
                # Parse command
                linear, angular = self.parse_command(command)
                
                # Handle quit command
                if linear == 'quit':
                    self.get_logger().info('👋 Goodbye!')
                    break
                
                # Skip invalid commands
                if linear is None:
                    continue
                    
                # Publish velocity command
                self.publish_velocity(linear, angular)
                
        except KeyboardInterrupt:
            self.get_logger().info('👋 Keyboard interrupt received. Goodbye!')
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')
        finally:
            # Send stop command before quitting
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('🛑 Sent final STOP command')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command
        try:
            node.publish_velocity(0.0, 0.0)
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()