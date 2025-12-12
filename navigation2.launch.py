import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get package directories
    kocbot_nav_dir = get_package_share_directory('kocbot_navigation2')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Set launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Normal map (for navigation)
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(kocbot_nav_dir, 'map', 'map.yaml'))

    # Filtered map (for costmap filtering)
    mask_yaml_file = LaunchConfiguration('mask', default=os.path.join(kocbot_nav_dir, 'map', 'map_filtered.yaml'))

    # Navigation parameters
    param_file = LaunchConfiguration('params_file', default=os.path.join(kocbot_nav_dir, 'param', 'kocbot.yaml'))

    # RViz configuration
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    # Rewritten YAML for costmap filter parameters
    configured_params = RewrittenYaml(
        source_file=param_file,
        root_key='',
        param_rewrites={'yaml_filename': mask_yaml_file},  # Ensure costmap filter uses filtered map
        convert_types=True
    )

    # Launch Description
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('map', default_value=map_yaml_file, description='Full path to normal map file'),
        DeclareLaunchArgument('params_file', default_value=param_file, description='Full path to parameter file'),
        DeclareLaunchArgument('mask', default_value=mask_yaml_file, description='Full path to filtered map'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),

        # Costmap Filter Nodes (Load Filtered Map)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[configured_params]  # Uses filtered map
        ),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[configured_params]  # Uses filtered map
        ),
        
        # Docking server as a regular Node (no lifecycle_nodes argument)
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[param_file],
        ),
        
        # Lifecycle Manager for Docking Server (Handles state transitions)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['docking_server']  # Automatically manages state transitions
            }]
        ),

        # Lifecycle Manager for Costmap Filters (Ensures that costmap filters are activated)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['filter_mask_server', 'costmap_filter_info_server']
            }]
        ),

        # Delay Navigation2 Launch to Ensure Costmap Filters Are Ready
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={'map': map_yaml_file, 'use_sim_time': use_sim_time, 'params_file': param_file}.items(),
        ),

        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])

