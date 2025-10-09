# triple_sensor_fusion.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ===================
        # Semua sensor nodes (LiDAR, Visual, IMU)
        # ===================
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='lidar_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom_lidar',
                'publish_tf': False,
                'wait_for_transform': 0.1,
                'Icp/PointToPlane': 'true',
                'Icp/MaxCorrespondenceDistance': '0.3',
            }],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom_lidar_raw'),
            ]
        ),

        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='visual_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom_visual',
                'publish_tf': False,
                'Odom/Strategy': '0',
                'Vis/MaxFeatures': '400',
            }],
            remappings=[
                ('rgb/image', '/dor/inpainted'),
                ('depth/image', '/dor/static_depth'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('odom', '/odom_visual_raw'),
            ]
        ),

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
                ('imu/data_raw', '/camera/imu'),
                ('imu/data', '/imu/filtered'),
            ]
        ),

        # ===================
        # EKF dengan 3 Sensor
        # ===================
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom_fusion',
            output='screen',
            parameters=[{
                'frequency': 100.0,
                'sensor_timeout': 0.05,
                
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                
                # LiDAR (utama untuk posisi 2D)
                'odom0': '/odom_lidar_raw',
                'odom0_config': [True,  True,  False,   # x, y, z
                                False, False, False,   # roll, pitch, yaw
                                True,  True,  False],  # vx, vy, vz
                'odom0_differential': False,
                
                # Visual (koreksi drift LiDAR)
                'odom1': '/odom_visual_raw',
                'odom1_config': [True,  True,  False,   # x, y, z (confidence rendah)
                                False, False, True,    # roll, pitch, yaw (confidence tinggi)
                                False, False, False],  # vx, vy, vz
                'odom1_differential': False,
                
                # IMU (utama untuk orientasi & angular velocity)
                'imu0': '/imu/filtered',
                'imu0_config': [False, False, False,   # x, y, z
                               True,  True,  True,    # roll, pitch, yaw
                               False, False, False,   # vx, vy, vz
                               True,  True,  True],   # ax, ay, az
                'imu0_differential': True,
                
                'two_d_mode': True,
            }],
            remappings=[
                ('odometry/filtered', '/odom'),
            ]
        ),

        # RTAB-Map node (sama seperti sebelumnya)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'subscribe_rgbd': False,
                'subscribe_scan': True,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom': True,
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'Reg/Strategy': '1',
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image', '/dor/inpainted'),
                ('depth/image', '/dor/static_depth'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('scan', '/scan'),
                ('odom', '/odom'),
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
    ])