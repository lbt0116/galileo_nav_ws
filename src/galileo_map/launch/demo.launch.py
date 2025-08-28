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

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('galileo_map'),
            'config',
            'galileo_map.rviz',
        ]),
        description='Path to RViz2 config file'
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='galileo_map_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        config_path_arg,
        rviz_config_arg,
        node,
        rviz_node,
    ])
