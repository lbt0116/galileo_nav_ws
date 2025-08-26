from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('galileo_map'),
            'config',
            'click_smooth_ros2.yaml',
        ]),
        description='Path to ROG-Map config yaml'
    )

    node = Node(
        package='galileo_map',
        executable='galileo_map_node',
        name='galileo_map_node',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path')
        }]
    )

    return LaunchDescription([config_path_arg, node])
