from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
