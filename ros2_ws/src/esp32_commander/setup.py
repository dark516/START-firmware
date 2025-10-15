from setuptools import setup
import os
from glob import glob

package_name = 'esp32_commander'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Копируем все launch файлы из папки launch
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='egor',
    maintainer_email='egor@example.com',
    description='ESP32 WiFi Bridge with RPLidar support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_bridge = esp32_commander.wifi_bridge:main',
            'robot_localization_node = esp32_commander.robot_localization_node:main',
            'lidar_bridge = esp32_commander.lidar_bridge:main',
        ],
    },
)