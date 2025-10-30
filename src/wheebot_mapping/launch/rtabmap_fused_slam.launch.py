import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    parameters=[{
        "use_sim_time": True,
        "subscribe_depth": True,
        "subscribe_rgbd": False,
        "subscribe_rgb": True,
        "subscribe_stereo": False,
        "subscribe_scan": False,
        "subscribe_scan_cloud": False,
        "subscribe_odom": True,
        "frame_id": "base_link",
        "map_frame_id": "map",
        "odom_frame_id": "", # If set, TF is used to get odometry instead of the topic.
        "publish_tf": True,
        "use_action_for_goal": False,
        "wait_for_transform": 0.5,
        "approx_sync": True,
        "topic_queue_size": 30,
        "sync_queue_size": 30,

        # # for saving and loading map, don't forget to delete arguments '--delete_db_on_start'
        # "database_path": os.path.join(get_package_share_directory('wheebot_mapping'), 'rtabmap.db'),
        # "database_path": "~/.ros/rtabmap.db",
    }]

    remappings=[
        ('map', '/map'),
        ('rgb/image', '/camera/image'),
        ('depth/image', '/camera/depth_image'),
        ('rgb/camera_info', '/camera/camera_info'),
        # ('scan', '/scan_for_slam'),
        # ('scan_cloud_topic', '/dor/dynamic_removed/pointcloud'),
        ('odom', '/odometry/filtered'),
        ('imu', '/camera/imu'),
    ]

    return LaunchDescription([
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=[
                '-d',
                '--delete_db_on_start'
            ]),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])
