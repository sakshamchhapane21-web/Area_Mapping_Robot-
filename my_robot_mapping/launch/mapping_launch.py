import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_mapping'
    pkg_dir = get_package_share_directory(pkg_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    bridge_config_path = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    sdf_file = LaunchConfiguration('sdf_file')
    
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', sdf_file],
        output='screen'
    )
    
    start_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'],
        output='screen'
    )

    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.3', '0', '0', '0', 'vehicle_blue/chassis', 'vehicle_blue/chassis/gpu_lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'vehicle_blue/chassis', 'vehicle_blue/chassis/imu_sensor'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Cartographer SLAM Node
    start_cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'mapping.lua',
        ]
    )
    
    start_occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # Nav2 Navigation Stack (Excluding AMCL and Map Server because Cartographer is doing SLAM)
    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_path
        }.items()
    )
    
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('sdf_file', default_value='main.sdf', description='Path to SDF world'),
        start_gazebo,
        start_bridge,
        static_tf_lidar,
        static_tf_imu,
        start_cartographer,
        start_occupancy_grid,
        start_nav2,
        start_rviz
    ])
