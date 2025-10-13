# ESP32 Commander with Robot Localization

This ROS2 package provides communication with your ESP32-based robot and calculates the robot's position and orientation using encoder and gyroscope data.

## Features

- **ESP32 Communication**: Receives encoder and gyro data via WiFi bridge
- **Robot Localization**: Calculates X/Y position and orientation using differential drive odometry
- **Real-time Publishing**: Publishes odometry data compatible with ROS2 navigation stack
- **Visual Demo**: Includes demo script to visualize robot position and sensor data

## System Architecture

```
ESP32 (WiFi) → Commander Node → Sensor Topics → Localization Node → Position Data
```

### Data Flow
1. **ESP32** sends: `left_ticks right_ticks yaw_degrees`
2. **Commander Node** publishes: `/encoder_left`, `/encoder_right`, `/gyro_yaw`
3. **Localization Node** calculates and publishes: `/odom`, `/robot_pose2d`, TF transforms

## Topics

### Published Topics
- `/encoder_left` (std_msgs/Int32): Left wheel encoder ticks
- `/encoder_right` (std_msgs/Int32): Right wheel encoder ticks  
- `/gyro_yaw` (std_msgs/Int32): Gyroscope yaw angle in degrees
- `/odom` (nav_msgs/Odometry): Full odometry data with covariance
- `/robot_pose2d` (geometry_msgs/Pose2D): Simple 2D pose (X, Y, theta)

### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands sent to robot

### TF Frames
- `odom` → `base_link`: Robot position in odometry frame

## Quick Start

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select esp32_commander
source install/setup.bash
```

### 2. Configure Robot Parameters
Edit the launch file or use parameters to set your robot's physical dimensions:

```bash
# Example parameters (adjust for your robot)
wheel_separation: 0.3      # Distance between wheels (meters)
wheel_radius: 0.065        # Wheel radius (meters)  
ticks_per_revolution: 1440 # Encoder ticks per wheel revolution
```

### 3. Start the System
```bash
# Start both commander and localization nodes
ros2 launch esp32_commander robot_system.launch.py

# Or start nodes individually:
ros2 run esp32_commander commander_node
ros2 run esp32_commander robot_localization_node
```

### 4. Monitor Robot Position
```bash
# View position data
ros2 topic echo /robot_pose2d

# View full odometry
ros2 topic echo /odom

# Run the visual demo
cd ~/ros2_ws/src/esp32_commander/scripts
python3 robot_position_demo.py
```

## Configuration

### ESP32 Connection
```bash
# Change ESP32 IP address
ros2 launch esp32_commander robot_system.launch.py esp32_host:=192.168.1.100

# Change port
ros2 launch esp32_commander robot_system.launch.py esp32_port:=3333
```

### Robot Physical Parameters
```bash
# Adjust for your robot's dimensions
ros2 launch esp32_commander robot_system.launch.py \
    wheel_separation:=0.25 \
    wheel_radius:=0.05 \
    ticks_per_revolution:=1000
```

## Usage Examples

### Basic Teleoperation
```bash
# Install teleop tools if needed
sudo apt install ros-humble-teleop-twist-keyboard

# Control robot with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View Robot in RViz
```bash
# Launch RViz with odometry visualization
rviz2 -d config/robot_viz.rviz
```

### Monitor System Status
```bash
# Check node status
ros2 node list
ros2 node info /esp32_commander
ros2 node info /robot_localization

# View topic rates
ros2 topic hz /odom
ros2 topic hz /encoder_left
```

## Calibration

### Wheel Parameters
1. **Measure wheel separation**: Distance between wheel contact points
2. **Measure wheel radius**: From center to ground contact
3. **Count encoder ticks**: Manually rotate wheel one full revolution

### Test Movement
1. Drive robot forward 1 meter
2. Compare odometry distance with actual distance
3. Adjust `wheel_radius` if needed

### Test Rotation  
1. Rotate robot 360 degrees
2. Compare odometry angle with actual rotation
3. Adjust `wheel_separation` if needed

## Troubleshooting

### No Position Data
- Check ESP32 connection: `ros2 topic echo /encoder_left`
- Verify robot parameters are correct
- Ensure both encoder topics are publishing

### Position Drift
- Calibrate wheel parameters
- Check for wheel slippage
- Consider gyro-based correction

### ESP32 Connection Issues
- Verify IP address and port
- Check WiFi connection
- Monitor commander node logs

### Build Issues
- Install dependencies: `rosdep install --from-paths . --ignore-src -r -y`
- Check Python path and permissions
- Ensure tf_transformations is installed: `pip3 install transforms3d`

## Files Structure

```
esp32_commander/
├── esp32_commander/
│   ├── commander_node.py          # ESP32 communication
│   └── robot_localization_node.py # Position calculation
├── launch/
│   └── robot_system.launch.py     # Launch both nodes
├── scripts/
│   └── robot_position_demo.py     # Visualization demo
├── package.xml                    # Package dependencies
├── setup.py                      # Package configuration
└── README.md                     # This file
```

## Parameters Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `esp32_host` | 192.168.125.241 | ESP32 IP address |
| `esp32_port` | 3333 | ESP32 TCP port |
| `wheel_separation` | 0.3 | Distance between wheels (m) |
| `wheel_radius` | 0.065 | Wheel radius (m) |
| `ticks_per_revolution` | 1440 | Encoder ticks per revolution |

## Development

### Adding New Features
1. Sensor fusion with gyroscope data
2. Kalman filter for better accuracy
3. Integration with ROS2 navigation stack
4. Map-based localization (AMCL)

### Contributing
1. Fork the repository
2. Create feature branch
3. Add tests and documentation
4. Submit pull request

## License

MIT License - See LICENSE file for details.