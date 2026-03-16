from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aeb_f110',
            executable='aeb_node',
            name='aeb_node',
            output='screen',
            parameters=[{
                'wheel_radius': 0.058,
                'range_min_cutoff': 0.13,
                'ttc_threshold': 0.61,
                'angular_window_deg': 12.0,
                'brake_command': 0.0,
            }],
        ),
    ])