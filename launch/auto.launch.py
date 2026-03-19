from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches the AEB safety node + the linear driver in autonomous mode.

    The linear driver publishes to /aeb_f110/sources/auto/{throttle,steering}.
    Those topics are remapped here directly to the AEB's input topics so that
    the AEB acts as the single safety gate — no other wiring is needed.

    When a multiplexer node is introduced (Option C), this remapping is removed
    and the multiplexer takes over the arbitration role.
    """
    return LaunchDescription([

        # --- AEB safety node (unchanged) ---
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

        # --- Linear driver (autonomous straight-line source) ---
        # Remapping bridges /aeb_f110/sources/auto/* → /aeb_f110/*_request
        # until the multiplexer node is added.
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
            remappings=[
                ('/aeb_f110/sources/auto/throttle', '/aeb_f110/throttle_request'),
                ('/aeb_f110/sources/auto/steering', '/aeb_f110/steering_request'),
            ],
        ),

    ])
