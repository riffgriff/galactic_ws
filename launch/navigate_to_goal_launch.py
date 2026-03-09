from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='galactic_navigate_to_goal',
            executable='go_to_goal',
            name='go_to_goal'
        ),
        Node(
            package='galactic_navigate_to_goal',
            executable='get_object_range',
            name='get_object_range'
        ),
        Node(
            package='galactic_navigate_to_goal',
            executable='drive_carefully',
            name='drive_carefully',
            parameters=[
                {'kp_r' : 1.8},
                {'ki_r' : 0.0},
                {'kd_r' : 0.0},
                {'kp_t' : 0.5},
                {'ki_t' : 0.0},
                {'kd_t' : 0.0},
                {'kp_obj' : 2.0},
                {'ki_obj' : 0.0},
                {'kd_obj' : 0.0},
                {'kp_pivot' : 0.3},
                {'ki_pivot' : 0.0},
                {'kd_pivot' : 0.0},
                {'avoidance_dist_threshold' : 0.4},
            ]
        ),
    ])
