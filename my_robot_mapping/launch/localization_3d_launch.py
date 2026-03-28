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
    
    bridge_config_path = os.path.join(pkg_dir, 'config', 'bridge_config_3d.yaml')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    pbstream_file = '/home/saksham3105/cartographer_ws/src/my_robot_mapping/maps/warehouse_map.pbstream'
    
    sdf_file = LaunchConfiguration('sdf_file')
    
    start_gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', sdf_file], output='screen')
    
    start_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'], output='screen'
    )

    static_tf_lidar = Node(package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0.3', '0', '0', '0', 'vehicle_blue/chassis', 'vehicle_blue/chassis/gpu_lidar'],
        parameters=[{'use_sim_time': True}])

    static_tf_imu = Node(package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'vehicle_blue/chassis', 'vehicle_blue/chassis/imu_sensor'],
        parameters=[{'use_sim_time': True}])

    static_tf_camera = Node(package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.5', '0', '0.05', '0', '0', '0', 'vehicle_blue/chassis', 'vehicle_blue/chassis/rgbd_camera'],
        parameters=[{'use_sim_time': True}])
    
    start_cartographer = Node(
        package='cartographer_ros', executable='cartographer_node',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'localization_3d.lua',
            '-load_state_filename', pbstream_file
        ], output='screen')
    
    start_occupancy_grid = Node(
        package='cartographer_ros', executable='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    # THE FIX IS HERE: Forced QoS overrides and opened Z-filters
    start_octomap = Node(
        package='octomap_server', executable='octomap_server_node',
        name='octomap_server',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05,
            'frame_id': 'map',
            'base_frame_id': 'vehicle_blue/chassis',
            'sensor_model.max_range': 8.0,
            'pointcloud_min_z': -1.0, 
            'pointcloud_max_z': 10.0,
            'qos_overrides./camera/points.subscription.reliability': 'best_effort' 
        }],
        remappings=[('cloud_in', '/camera/points')],
        output='screen')

    start_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'True', 'params_file': nav2_params_path}.items())
    
    start_rviz = Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('sdf_file', default_value='main.sdf'),
        start_gazebo, start_bridge,
        static_tf_lidar, static_tf_imu, static_tf_camera,
        start_cartographer, start_occupancy_grid, start_octomap,
        start_nav2, start_rviz
    ])
