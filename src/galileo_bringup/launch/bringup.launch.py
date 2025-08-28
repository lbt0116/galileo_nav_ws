from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # 读取 bringup.yaml 配置
    bringup_share = get_package_share_directory('galileo_bringup')
    cfg_file = os.path.join(bringup_share, 'config', 'bringup.yaml')
    with open(cfg_file, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['bringup']['ros__parameters']

    # 启动参数（从 bringup.yaml 读取默认值）
    use_klio_arg = DeclareLaunchArgument(
        'use_klio',
        default_value=str(cfg.get('use_klio', True)).lower(),
        description='Enable galileo_klio launch'
    )
    use_sam_arg = DeclareLaunchArgument(
        'use_sam',
        default_value=str(cfg.get('use_sam', True)).lower(),
        description='Enable galileo_sam launch'
    )
    use_map_arg = DeclareLaunchArgument(
        'use_map',
        default_value=str(cfg.get('use_map', False)).lower(),
        description='Enable galileo_map node'
    )
    use_bridge_arg = DeclareLaunchArgument(
        'use_bridge',
        default_value=str(cfg.get('use_bridge', False)).lower(),
        description='Enable LCM-ROS2 bridge node'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for KLIO'
    )
    use_bag_play_arg = DeclareLaunchArgument(
        'use_bag_play',
        default_value=str(cfg.get('use_bag_play', False)).lower(),
        description='Enable ros2bagplay'
    )
    bag_file_path_arg = DeclareLaunchArgument(
        'bag_file_path',
        default_value=cfg.get('bag_file_path', ''),
        description='Path to the bag file for ros2bagplay'
    )

    # 日志
    startup_info = LogInfo(msg='[Galileo Bringup] 正在启动 Galileo 导航系统...')

    # 引入 KLIO 的子 launch（不改动原包）
    klio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('galileo_klio'),
                'launch',
                'galileo_klio_launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_klio')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    klio_enabled_log = LogInfo(
        msg='[Galileo Bringup] galileo_klio 启用',
        condition=IfCondition(LaunchConfiguration('use_klio'))
    )
    klio_disabled_log = LogInfo(
        msg='[Galileo Bringup] galileo_klio 禁用',
        condition=UnlessCondition(LaunchConfiguration('use_klio'))
    )

    # 引入 SAM 的子 launch（默认启动其 RViz）
    sam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('galileo_sam'),
                'launch',
                'run.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_sam')),
        launch_arguments={
            'rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    sam_enabled_log = LogInfo(
        msg='[Galileo Bringup] galileo_sam 启用',
        condition=IfCondition(LaunchConfiguration('use_sam'))
    )
    sam_disabled_log = LogInfo(
        msg='[Galileo Bringup] galileo_sam 禁用',
        condition=UnlessCondition(LaunchConfiguration('use_sam'))
    )

    # ros2bagplay 节点 -> 使用进程方式运行 CLI
    bag_play_proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file_path')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bag_play'))
    )

    bag_play_enabled_log = LogInfo(
        msg='[Galileo Bringup] ros2bagplay 启用',
        condition=IfCondition(LaunchConfiguration('use_bag_play'))
    )
    bag_play_disabled_log = LogInfo(
        msg='[Galileo Bringup] ros2bagplay 禁用',
        condition=UnlessCondition(LaunchConfiguration('use_bag_play'))
    )

    # 可选：map 仍可通过直起 node 的方式（保留开关，默认关闭）
    # 这里为了"原包不动"，不包含 map 的子 launch；如未来提供 Python launch 再替换
    # from launch_ros.actions import Node  # 若需启用可按需导入
    # map_node = Node(
    #     package='galileo_map',
    #     executable='galileo_map_node',
    #     name='galileo_map_node',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('use_map'))
    # )
    map_enabled_log = LogInfo(
        msg='[Galileo Bringup] galileo_map 启用',
        condition=IfCondition(LaunchConfiguration('use_map'))
    )
    map_disabled_log = LogInfo(
        msg='[Galileo Bringup] galileo_map 禁用',
        condition=UnlessCondition(LaunchConfiguration('use_map'))
    )

    # 引入 galileo_map 的子 launch
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('galileo_map'),
                'launch',
                'demo.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_map')),
        launch_arguments={},
    )

    # LCM-ROS2 Bridge 节点
    bridge_node = Node(
        package='galileo_lcm_ros2_bridge',
        executable='recorder_node',
        name='lcm_ros2_bridge_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_bridge'))
    )

    bridge_enabled_log = LogInfo(
        msg='[Galileo Bringup] LCM-ROS2 bridge 启用',
        condition=IfCondition(LaunchConfiguration('use_bridge'))
    )
    bridge_disabled_log = LogInfo(
        msg='[Galileo Bringup] LCM-ROS2 bridge 禁用',
        condition=UnlessCondition(LaunchConfiguration('use_bridge'))
    )

    return LaunchDescription([
        # args
        use_klio_arg,
        use_sam_arg,
        use_map_arg,
        use_bridge_arg,
        use_rviz_arg,
        use_bag_play_arg,
        bag_file_path_arg,
        # logs
        startup_info,
        klio_enabled_log,
        klio_disabled_log,
        sam_enabled_log,
        sam_disabled_log,
        bag_play_enabled_log,
        bag_play_disabled_log,
        map_enabled_log,
        map_disabled_log,
        bridge_enabled_log,
        bridge_disabled_log,
        # includes
        klio_launch,
        sam_launch,
        map_launch,
        # nodes
        bag_play_proc,
        bridge_node,
        # 可选：map_node,
    ])
