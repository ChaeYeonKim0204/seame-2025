from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_camera_driver',
            executable='usb_camera_driver_node',
            namespace='/camera',
            name='usb_camera_node',
            output='screen',
            parameters=[
                {"camera_id": 0},  # /dev/video0
                {"image_width": 1280},
                {"image_height": 720},
                {"fps": 30.0},
                {"frame_id": "camera"},
                {"camera_calibration_file": ""}
            ]
        ),
        Node(
            package='lane5c_pkg',
            executable='lane5c_node',
            namespace='/lane5c',
            name='lane5c_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='control',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
        Node(
            package='control',
            executable='manual_control_node',
            name='manual_control_node',
            output='screen'
        ),
        Node(
            package='control',
            executable='mode_switch_node',
            name='mode_switch_node',
            output='screen'
        ),
    ])