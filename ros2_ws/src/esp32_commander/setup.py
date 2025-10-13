from setuptools import setup
import os
from glob import glob

package_name = 'esp32_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
        install_requires=['setuptools', 'tf_transformations'],
    zip_safe=True,
    maintainer='egor',
    maintainer_email='egor@example.com',
    description='ESP32 bridge for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_node = esp32_commander.commander_node:main',
            'robot_localization_node = esp32_commander.robot_localization_node:main',
            'wifi_bridge = esp32_commander.wifi_bridge:main',
            'keyboard_teleop = esp32_commander.keyboard_teleop_node:main',
        ],
    },
)

