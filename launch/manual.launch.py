from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Manual (teleop) mode: AEB + multiplexer + mode switcher.

    The linear driver is NOT launched. The active source defaults to 'teleop'.

    Launch teleop in a separate terminal, remapped to the teleop source namespace:
        ros2 run autodrive_f1tenth teleop_keyboard --ros-args \\
          -r /autodrive/f1tenth_1/throttle_command:=/aeb_f110/sources/teleop/throttle \\
          -r /autodrive/f1tenth_1/steering_command:=/aeb_f110/sources/teleop/steering

    Press [A] in the mode_switcher terminal to enable autonomous (linear_driver must
    also be running) or [T] to return to teleop.
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

        # --- Mode switcher (keyboard interface) ---
        Node(
            package='aeb_f110',
            executable='mode_switcher_node',
            name='mode_switcher_node',
            prefix='xterm -e',
        ),

    ])
