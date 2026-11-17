from setuptools import setup
import os
from glob import glob

package_name = 'jetson_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seif A. ElHadidy',
    maintainer_email='hadidyy@users.noreply.github.com',
    description='Full-stack autonomous driver for Jetson Orin Nano + 2 ESP32s.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node         = jetson_driver.camera_node:main',
            'perception_node     = jetson_driver.perception_node:main',
            'planner_node        = jetson_driver.planner_node:main',
            'control_node        = jetson_driver.control_node:main',
            'motor_bridge_node   = jetson_driver.motor_bridge_node:main',
            'trigger_bridge_node = jetson_driver.trigger_bridge_node:main',
        ],
    },
)
