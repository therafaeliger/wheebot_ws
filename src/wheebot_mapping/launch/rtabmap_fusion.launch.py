from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        # ===================
        # General arguments
        # ===================
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db'),

        # ===================
        # RTAB-Map SLAM node
        # ===================
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'subscribe_rgbd': False,
                'subscribe_scan': True,
                'subscribe_scan_cloud': False,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_imu': True,
                'icp_odometry': True,
                'visual_odometry': False,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'publish_tf': True,
                'database_path': LaunchConfiguration('database_path'),

                'approx_sync': True,
                'topic_queue_size': 100,
                'sync_queue_size': 100,
                'approx_sync_max_interval': 1.0,
            }],
            remappings=[
                ('rgb/image', '/dor/static_rgb'),
                ('depth/image', '/dor/static_depth'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('scan', '/scan_for_slam'),
                ('odom', '/odom'),
                ('imu', '/imu/data')
            ]
        ),

        # ===================
        # ICP Odometry (LiDAR)
        # ===================
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'wait_for_transform': 0.2,

                'approx_sync': True,
                'topic_queue_size': 100,
                'sync_queue_size': 100,
                'approx_sync_max_interval': 1.0,
            }],
            remappings=[
                ('scan', '/scan_for_slam'),
                ('odom', '/odom'),
                ('imu', '/imu/data')
            ]
        ),

        # ===================
        # Visualization
        # ===================
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'subscribe_rgbd': False,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': True,
                'subscribe_imu': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                
                'approx_sync': True,
                'topic_queue_size': 100,
                'sync_queue_size': 100,
                'approx_sync_max_interval': 1.0,
            }],
            remappings=[
                ('rgb/image', '/dor/static_rgb'),
                ('depth/image', '/dor/static_depth'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('scan', '/scan_for_slam'),
                ('odom', '/odom'),
                ('imu', '/imu/data')
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.63', '0', '0', '0', 'base_link', 'camera_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.7', '3.14159265', '0', '0', 'base_link', 'laser_frame']
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_a1',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',   # ubah sesuai port
                'serial_baudrate': 115200,       # default RPLidar A1
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True
            }]
        ),

        # Launch arguments
        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('realsense2_camera'), 'launch'),
        #         '/rs_launch.py']),
        #         launch_arguments={'camera_namespace': '',
        #                           'enable_gyro': 'true',
        #                           'enable_accel': 'true',
        #                           'unite_imu_method': LaunchConfiguration('unite_imu_method'),
        #                           'align_depth.enable': 'true',
        #                           'enable_sync': 'true',
        #                           'rgb_camera.profile': '640x360x30'}.items(),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('realsense2_camera'), 'examples', 'align_depth'),
        #         '/rs_align_depth_launch.py']),
        # ),

        # Compute quaternion from Realsense IMU (Madgwick Filter)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,         # kita pakai accel+gyro saja
                'world_frame': 'enu',     # East-North-Up, standar ROS
                'publish_tf': False
            }],
            remappings=[
                ('imu/data_raw', '/camera/imu'),   # input dari realsense
                ('imu/data', '/imu/data')          # output dengan orientation
            ]
        )

    ])
