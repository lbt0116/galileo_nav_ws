from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('galileo_sam')

    rviz = LaunchConfiguration('rviz')
    params_file = os.path.join(pkg_share, 'config', 'config.yaml')

    ld = []

    ld.append(DeclareLaunchArgument('rviz', default_value='true'))

    def launch_setup(context, *args, **kwargs):
        nodes = []

        nodes.append(
            Node(
                package='galileo_sam',
                executable='galileo_sam_node',
                name='galileo_sam_node',
                output='screen',
                parameters=[params_file]
            )
        )

        if rviz.perform(context) == 'true':
            nodes.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz_galileo',
                    output='log'
                )
            )

        return nodes

    ld.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(ld)


