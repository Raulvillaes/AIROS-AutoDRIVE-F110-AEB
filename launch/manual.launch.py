from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Manual (teleop) mode: AEB + multiplexer.

    The linear driver is NOT launched. The active source defaults to 'teleop'.

    Run each of the following in a separate terminal:

        ros2 run autodrive_f1tenth teleop_keyboard --ros-args \\
          -r /autodrive/f1tenth_1/throttle_command:=/aeb_f110/sources/teleop/throttle \\
          -r /autodrive/f1tenth_1/steering_command:=/aeb_f110/sources/teleop/steering

        ros2 run aeb_f110 mode_switcher_node
    """
    return LaunchDescription([

        # --- AEB safety node ---
        Node(
            package='aeb_f110',
            executable='aeb_node',
            name='aeb_node',
            output='screen',
            parameters=[{
                'wheel_radius':       0.058,
                'range_min_cutoff':   0.13,
                'ttc_threshold':      0.66,
                'angular_window_deg': 12.5,
                'brake_command':      0.0,
                'min_speed':          0.8,
                'encoder_timeout':    0.5,
                'max_wheel_speed':    8.0,
            }],
        ),

        # --- Multiplexer (teleop active by default) ---
        Node(
            package='aeb_f110',
            executable='mux_node',
            name='mux_node',
            output='screen',
            parameters=[{
                'active_source':  'teleop',
                'source_timeout': 0.5,
            }],
        ),

    ])
