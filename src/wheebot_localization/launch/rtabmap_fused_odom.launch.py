import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # add directory path
    wheebot_localization_dir = get_package_share_directory('wheebot_localization')
    ekf_config = os.path.join(wheebot_localization_dir, 'config', 'ekf.yaml')

    # add sim time arg
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # visual inertial odom
    vio_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_vio',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": "base_link",
            "odom_frame_id": "odom_vio",
            "publish_tf": False,
            "wait_for_transform": 0.2,
            "wait_imu_to_init": True,
            "always_check_imu_tf": False,
            "approx_sync": True,
            "topic_queue_size": 15,
            "sync_queue_size": 15,
            "approx_sync_max_interval": 0.05,
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

    # lidar inertial odom
    lio_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='rtabmap_lio',
        output='screen',
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": "base_link",
            "odom_frame_id": "odom_lio",
            "publish_tf": False,
            "wait_for_transform": 0.2,
            "wait_imu_to_init": True,
            "always_check_imu_tf": False,
            "approx_sync": True,
            "topic_queue_size": 15,
            "sync_queue_size": 15,
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

    # fusing odometry using ekf
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time},
            ekf_config,
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        vio_node,
        lio_node,
        ekf_node,
    ])
