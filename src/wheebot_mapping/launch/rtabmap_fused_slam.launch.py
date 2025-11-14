from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # add launch arg
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    localization_arg = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Launch in localization mode.')
    localization = LaunchConfiguration('localization')
    
    # add params and remappings
    parameters={
        "use_sim_time": use_sim_time,
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
        "wait_for_transform": 0.2,
        "approx_sync": True,
        "topic_queue_size": 15,
        "sync_queue_size": 15,
        "approx_sync_max_interval": 0.05,

        "Reg/Force3DoF":"true",
        "Grid/RayTracing":"true",
        "Grid/3D":"false",
        "Grid/RangeMax":"3",
        "Grid/NormalsSegmentation":"false",
        "Grid/MaxGroundHeight":"0.05",
        "Grid/MaxObstacleHeight":"0.4",
        "Optimizer/GravitySigma":"0",

        # # for saving and loading map, don't forget to delete arguments '--delete_db_on_start'
        # "database_path": os.path.join(get_package_share_directory('wheebot_mapping'), 'rtabmap.db'),
        # "database_path": "~/.ros/rtabmap.db",
    }

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

    slam_node = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d']
    )

    localization_node = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters,
            {'Mem/IncrementalMemory':'false',
            'Mem/InitWMWithAllNodes':'true'}],
        remappings=remappings
    )

    viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters],
        remappings=remappings
    )

    return LaunchDescription([
        use_sim_time_arg,
        localization_arg,
        
        # SLAM mode:
        slam_node,

        # Localization mode:
        localization_node,

        viz_node,
    ])
