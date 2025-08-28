from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='galileo_lcm_ros2_bridge',
            executable='recorder_node',
            name='recorder_node',
            output='screen',
            parameters=[{
                'lcm_suffix': 'galileo',
                'lcm_port': 20000,
                'lcm_ttl': 1,
            }]
        )
    ])


