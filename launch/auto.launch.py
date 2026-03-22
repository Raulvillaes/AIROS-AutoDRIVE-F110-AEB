from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Autonomous mode: AEB + multiplexer + linear driver.

    Data flow:
        linear_driver_node → /aeb_f110/sources/auto/*
                                  ↓
                            mux_node  ←  mode_switcher_node (separate terminal)
                                  ↓
                            /aeb_f110/throttle_request  →  aeb_node  →  simulator
                            /aeb_f110/steering_request

    Run in separate terminals:
        ros2 run aeb_f110 mode_switcher_node

        ros2 run autodrive_f1tenth teleop_keyboard --ros-args \\
          -r /autodrive/f1tenth_1/throttle_command:=/aeb_f110/sources/teleop/throttle \\
          -r /autodrive/f1tenth_1/steering_command:=/aeb_f110/sources/teleop/steering
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

        # --- Multiplexer ---
        Node(
            package='aeb_f110',
            executable='mux_node',
            name='mux_node',
            output='screen',
            parameters=[{
                'active_source':  'auto',
                'source_timeout': 0.5,
            }],
        ),

        # --- Linear driver (autonomous straight-line source) ---
        Node(
            package='aeb_f110',
            executable='linear_driver_node',
            name='linear_driver_node',
            output='screen',
            parameters=[{
                'throttle':     0.3,
                'steering':     0.0,
                'publish_rate': 20.0,
            }],
        ),

    ])
