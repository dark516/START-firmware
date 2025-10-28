#!/bin/bash
# Test SLLIDAR with TCP connection to ESP32

cd /home/egor/Desktop/project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Testing SLLIDAR with TCP connection..."
ros2 run sllidar_ros2 sllidar_node --ros-args \
    -p channel_type:=tcp \
    -p tcp_ip:=192.168.1.72 \
    -p tcp_port:=3333 \
    -p frame_id:=laser \
    -p scan_frequency:=10.0