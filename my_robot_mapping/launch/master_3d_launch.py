import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_mapping')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    bridge_config_path = os.path.join(pkg_dir, 'config', 'bridge_config_3d.yaml')
    cartographer_config_dir = os.path.join(pkg_dir, 'config')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    pbstream_file = '/home/saksham3105/cartographer_ws/src/my_robot_mapping/maps/warehouse_map.pbstream'
    
    sdf_file = LaunchConfiguration('sdf_file')
    
    return LaunchDescription([
        DeclareLaunchArgument('sdf_file', default_value='main.sdf'),
        
        # 1. Gazebo Simulation
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', sdf_file], output='screen'),
        
        # 2. Bridge
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['--ros-args', '-p', f'config_file:={bridge_config_path}'], output='screen'),

        # 3. TF Tree (Robot Anatomy)
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.3', '0', '0', '0', 'vehicle_blue/chassis', 'vehicle_blue/chassis/gpu_lidar'],
             parameters=[{'use_sim_time': True}]),
             
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'vehicle_blue/chassis', 'vehicle_blue/chassis/imu_sensor'],
             parameters=[{'use_sim_time': True}]),

        # Forward-Facing Camera Transformation
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.5', '0', '0.05', '0', '0', '0', 'vehicle_blue/chassis', 'vehicle_blue/chassis/rgbd_camera'],
             parameters=[{'use_sim_time': True}]),

        # 4. Cartographer (2D SLAM, Localization, & Kidnapping Recovery)
        Node(package='cartographer_ros', executable='cartographer_node',
             parameters=[{'use_sim_time': True}],
             arguments=['-configuration_directory', cartographer_config_dir,
                        '-configuration_basename', 'localization_3d.lua',
                        '-load_state_filename', pbstream_file]),
                        
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             parameters=[{'use_sim_time': True}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),

        # 5. RTAB-Map (Passive 3D Memory Builder)
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             parameters=[{
                 'use_sim_time': True,
                 'frame_id': 'vehicle_blue/chassis',
                 'map_frame_id': 'map',
                 'odom_frame_id': 'map',
                 'publish_tf': False,
                 'subscribe_imu': True,            
                 'subscribe_depth': False,
                 'subscribe_rgb': False,
                 'subscribe_scan_cloud': True,   
                 'approx_sync': True,
                 'Grid/3D': 'true',              
                 'Grid/RayTracing': 'true',      
                 'RGBD/LinearUpdate': '0.1',     
                 'RGBD/AngularUpdate': '0.1',
                 'Reg/Strategy': '0',            # 0 = Turn off RTAB-Map math. Blindly trust Cartographer!
                 'Kp/MaxFeatures': '-1'          # -1 = Disable Visual Loop Closures to save CPU!
             }],
             remappings=[
                 ('scan_cloud', '/camera/points'),
                 ('imu', '/imu'),
                 # THE FIX: Quarantine RTAB-Map's 2D maps so they don't overwrite Cartographer!
                 ('map', '/rtabmap/quarantine_map'),
                 ('grid_map', '/rtabmap/quarantine_grid')
             ], output='screen'),

        # 6. Nav2 Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'True', 'params_file': nav2_params_path}.items()),
        
        # 7. RViz Visualization
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}], output='screen')
    ])
