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
                'range_min_cutoff': 0.07,
                'ttc_threshold': 0.6,
                'range_floor': 0.3,
                'range_floor_min_speed': 0.7,
                'angular_window_deg': 45.0,
                'range_floor_angular_window_deg': 30.0,

                'brake_command': 0.0,
                'brake_stop_speed': 0.05,
            }],
        ),
    ])

    # To use with teleop, run in a separate terminal:
    #
    #   ros2 run autodrive_f1tenth teleop_keyboard --ros-args \
    #     -r /autodrive/f1tenth_1/throttle_command:=/aeb_f110/throttle_request \
    #     -r /autodrive/f1tenth_1/steering_command:=/aeb_f110/steering_request
