import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    wheebot_localization_dir = get_package_share_directory('wheebot_localization')
    ekf_config = os.path.join(wheebot_localization_dir, 'config', 'ekf.yaml')

    vio_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_vio',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wait_imu_to_init': True,
            'approx_sync': True,
            'publish_tf': True,
            'frame_id': 'base_link',
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'subscribe_imu': True,
            'odom_frame_id': 'odom_vio',
        }],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('depth/image', '/camera/depth_image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', '/imu/data'),
            ('odom', '/vio/odom'),
        ],
    )

    lio_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='rtabmap_lio',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'wait_imu_to_init': True,
            'approx_sync': True,
            'publish_tf': True,
            'frame_id': 'base_link',
            'subscribe_scan': True,
            'subscribe_scan_cloud': False,
            'subscribe_imu': True,
            'odom_frame_id': 'odom_lio',
        }],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/imu/data'),
            ('odom', '/lio/odom'),
        ],
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False
        }],
        remappings=[
            ('imu/data_raw', '/camera/imu'),
            ('imu/data', '/imu/data')
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config],
    )

    return LaunchDescription([
        imu_filter,
        vio_node,
        lio_node,
        ekf_node,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.3616', '0.2157', '0.63', '0', '0', '0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.3036', '0.2157', '0.7', '0', '0', '0', 'base_link', 'laser_link']
        ),
    ])
