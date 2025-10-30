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
            "use_sim_time": True,
            "frame_id": "base_link",
            "odom_frame_id": "odom_vio",
            "publish_tf": True,
            "wait_for_transform": 0.2,
            "wait_imu_to_init": True,
            "always_check_imu_tf": False,
            "approx_sync": True,
            "topic_queue_size": 30,
            "sync_queue_size": 30,
            "subscribe_rgbd": False,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            'subscribe_imu': True,
        }],
        remappings=[
            ('rgb/image', '/camera/image'),
            ('depth/image', '/camera/depth_image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('odom', '/odom/vio'),
            ('imu', '/camera/imu'),
        ],
    )

    lio_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='rtabmap_lio',
        output='screen',
        parameters=[{
            "use_sim_time": True,
            "frame_id": "base_link",
            "odom_frame_id": "odom_lio",
            "publish_tf": True,
            "wait_for_transform": 0.2,
            "wait_imu_to_init": True,
            "always_check_imu_tf": False,
            "approx_sync": True,
            "topic_queue_size": 30,
            "sync_queue_size": 30,
            "subscribe_scan": True,
            "subscribe_scan_cloud": False,
            'subscribe_imu': True,
        }],
        remappings=[
            ('scan', '/scan'), # if u use lidar_range.py change to /scan_for_slam
            # ('/scan_cloud', '/dor/dynamic_removed/pointcloud'),
            ('odom', '/odom/lio'),
            ('imu', '/camera/imu'),
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
        vio_node,
        lio_node,
        ekf_node,
    ])
