"""Launch file for the full jetson_driver stack.

Brings up, all concurrent (no inter-node timers needed):
    camera_node          -> /front_camera (+ /rear_camera if enabled)
    perception_node      -> /lane_error, /obstacle_ahead, /yolo_blindspot_right
    planner_node         -> /target_speed, /behavior, /drive_enabled
    control_node         -> /throttle_cmd, /steer_cmd, /brake_cmd
    motor_bridge_node    -> serial TX to the motor ESP32
    trigger_bridge_node  <- WiFi TCP server for the trigger/awareness ESP32

Usage on the Jetson:

    source /opt/ros/humble/setup.bash
    source ~/jetson_ws/install/setup.bash
    ros2 launch jetson_driver jetson_driver.launch.py

Parameters live in config/driver_params.yaml and are loaded into every
node automatically.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('jetson_driver')
    default_params = os.path.join(pkg, 'config', 'driver_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the driver_params.yaml for all nodes')
    params = LaunchConfiguration('params_file')

    def _node(name, exe=None):
        return Node(
            package='jetson_driver',
            executable=exe or name,
            name=name,
            output='screen',
            parameters=[params],
            emulate_tty=True,
        )

    # Optional: serve /perception_debug over HTTP via web_video_server so a
    # phone or laptop on the same WiFi can watch the live overlay.
    web_video = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080, 'address': '0.0.0.0'}],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        _node('camera_node'),
        _node('perception_node'),
        _node('planner_node'),
        _node('control_node'),
        _node('motor_bridge_node'),
        _node('trigger_bridge_node'),
        web_video,
    ])
