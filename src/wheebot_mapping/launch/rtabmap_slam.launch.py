import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # wheebot_mapping_dir = get_package_share_directory("wheebot_mapping")
    rtabmap_launch_dir = get_package_share_directory("rtabmap_launch")

    rtabmap_launch = IncludeLaunchDescription(
        os.path.join(rtabmap_launch_dir, "launch", "rtabmap.launch.py"),
        launch_arguments={
            'use_sim_time': 'false',
            'args': '--delete_db_on_start',
            # 'args': '--database_path ~/.ros/rtabmap.db',
            'rtabmap_viz': 'true',
            'rviz': 'true',

            'stereo': 'false',
            'localization': 'false',
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'map_topic': 'map',
            'publish_tf_map': 'true',
            'namespace': 'rtabmap',
            # 'database_path': '~/.ros/rtabmap.db',
            'topic_queue_size': '10',
            'queue_size': '10',
            'wait_for_transform': '1.0',
            'approx_sync': 'true',
            'approx_rgbd_sync': 'true',
            'approx_sync_max_interval': '0.05',

            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/dor/dynamic_removed/depth',
            'camera_info_topic': '/camera/color/camera_info',

            'subscribe_rgbd': 'false',

            'subscribe_scan': 'false',
            'scan_topic': '/scan_for_slam',
            'subscribe_scan_cloud': 'true',
            'scan_cloud_topic': '/dor/dynamic_removed/pointcloud',

            'visual_odometry': 'true',
            'icp_odometry': 'false',
            'odom_topic': 'odom',
            'publish_tf_odom': 'true',

            'imu_topic': '/imu/data',
            'wait_imu_to_init': 'true',
            'always_check_imu_tf': 'false',

            # 'qos': '0',
            
        }.items()
    )

    return LaunchDescription([
        rtabmap_launch,      
    ])