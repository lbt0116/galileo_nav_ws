#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='galileo_klio_params.yaml',
        description='Path to the config file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz if true'
    )
    
    # 获取包路径
    pkg_share = FindPackageShare('galileo_klio')
    
    # 配置文件路径
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        LaunchConfiguration('config_file')
    ])
    
    # RViz配置文件路径，已修改为 rviz/galileo_klio.rviz
    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'galileo_klio.rviz'
    ])
    
    # 创建节点
    galileo_klio_node = Node(
        package='galileo_klio',
        executable='galileo_klio_node',
        name='galileo_klio_node',
        # prefix='gdb -ex run --args',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        use_rviz_arg,
        galileo_klio_node,
        rviz_node
    ]) 