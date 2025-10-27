import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch Intel RealSense D435i
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
                launch_arguments={'camera_namespace': '',
                                  'enable_gyro': 'true',
                                  'enable_accel': 'true',
                                  'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                                  'align_depth.enable': 'true',
                                  'enable_sync': 'true',
                                  'rgb_camera.profile': '640x360x30'}.items(),
        ),
        SetParameter(name='depth_module.emitter_enabled', value=1),
        
        # IMU Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
            }],
            remappings=[
                ('imu/data', '/imu/data'),
                ('imu/data_raw', '/camera/imu'),
            ],
        ),

        # Launch RPLiDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_a1',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', # periksa kalau pakai mikon
                'serial_baudrate': 115200,
                'frame_id': 'laser_link',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),

        # # Static TF
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0.3616', '0.2157', '0.63', '0', '0', '0', 'base_link', 'camera_link']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0.3036', '0.2157', '0.7', '3.1416', '0', '0', 'base_link', 'laser_link']
        # ),

    ])
