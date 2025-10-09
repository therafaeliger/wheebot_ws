import os
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Konfigurasi untuk direktori penting
    wheebot_mapping_dir = get_package_share_directory("wheebot_mapping")
    rtabmap_launch_dir = get_package_share_directory("rtabmap_launch")
    config_rviz = os.path.join(wheebot_mapping_dir, 'rviz', 'rtabmap_slam.rviz')

    # Launch arguments yang digunakan
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_load_argument = DeclareLaunchArgument(
        'load_map',
        default_value='false',
        description='Mapping mode: "false" for new mapping and save it, "true" for loading existing map'
    )

    declare_unite_imu_argument = DeclareLaunchArgument(
        'unite_imu_method', default_value='2',
        description='0-None, 1-copy, 2-linear_interpolation. '
                    'Use unite_imu_method:=1 if IMU topics stop being published.'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    load_map = LaunchConfiguration('load_map')
    

    # Args untuk RTABMap base
    rtabmap_base_args = {
        'namespace': '',
        'rtabmap_viz': 'true',
        'rviz': 'true',
        'rviz_cfg': config_rviz,

        # define mode
        'localization': 'false',
        'icp_odometry': 'false',
        'visual_odometry': 'true',
        'publish_tf_map': 'true',
        'publish_tf_odom': 'true',
        "wait_imu_to_init": "true",

        # sync settings
        'approx_sync': 'false',
        'rgbd_sync': 'true',
        'approx_rgbd_sync': 'false',

        # subscribe parameter
        'depth': 'true',
        'stereo': 'false',
        'subscribe_rgbd': 'false',
        'subscribe_rgb': 'false',
        'subscribe_scan': 'false',
        'subscribe_scan_cloud': 'false',
    }

    # Args untuk robot launch
    rtabmap_robot_args = {
        **rtabmap_base_args,

        # frame
        'frame_id': 'camera_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',

        # topics
        'rgb_topic': '/camera/color/image_raw',
        'depth_topic': '/camera/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera/color/camera_info',
        'odom_topic': '/odom',
        'imu_topic': '/imu/data',
    }
    rtabmap_robot_save_args = {**rtabmap_robot_args, 'args': '--delete_db_on_start'}
    rtabmap_robot_load_args = {**rtabmap_robot_args, 'args': '--database_path ~/.ros/rtabmap.db'}
    
    # Args untuk sim launch
    rtabmap_sim_args = {
        **rtabmap_base_args,

        # frame
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',

        # topics
        'rgb_topic': '/camera/image',
        'depth_topic': '/camera/depth_image',
        'camera_info_topic': '/camera/camera_info',
        'odom_topic': '/odom',
        'imu_topic': '/imu/out',
    }
    rtabmap_sim_save_args = {**rtabmap_sim_args, 'args': '--delete_db_on_start'}
    rtabmap_sim_load_args = {**rtabmap_sim_args, 'args': '--database_path ~/.ros/rtabmap.db'}

    # --- RealSense Camera Launch ---
    rs_launch = lambda: IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py'
        ]),
        launch_arguments={
            'camera_namespace': '',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'rgb_camera.profile': '640x360x30'
        }.items(),
    )

    # launch rtabmap robot action
    robot_action = GroupAction(
        condition=UnlessCondition(use_sim_time),
        actions=[
            GroupAction(
                condition=IfCondition(load_map),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, "launch", "rtabmap.launch.py")),
                        launch_arguments=rtabmap_robot_load_args.items(),
                    ),
                    rs_launch(),
                ]
            ),
            GroupAction(
                condition=UnlessCondition(load_map),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, "launch", "rtabmap.launch.py")),
                        launch_arguments=rtabmap_robot_save_args.items(),
                    ),
                    rs_launch(),
                ]
            )
        ]
    )

    # launch rtabmap sim action
    sim_action = GroupAction(
        condition=IfCondition(use_sim_time),
        actions=[
            GroupAction(
                condition=IfCondition(load_map),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, "launch", "rtabmap.launch.py")),
                        launch_arguments=rtabmap_sim_load_args.items(),
                    )
                ]
            ),
            GroupAction(
                condition=UnlessCondition(load_map),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, "launch", "rtabmap.launch.py")),
                        launch_arguments=rtabmap_sim_save_args.items(),
                    )
                ]
            )
        ]
    )

    # for rtabmap run manually
    # parameters=[{
    #       'frame_id':'camera_link',
    #       'subscribe_depth':True,
    #       'subscribe_odom_info':True,
    #       'approx_sync':False,
    #       'wait_imu_to_init':True}]

    # remappings=[
    #       ('imu', '/imu/data'),
    #       ('rgb/image', '/camera/color/image_raw'),
    #       ('rgb/camera_info', '/camera/color/camera_info'),
    #       ('depth/image', '/camera/aligned_depth_to_color/image_raw')]


    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_load_argument,
        declare_unite_imu_argument,

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch RTABMap
        robot_action,
        sim_action,

        # Run RTABMap manually
        # Node(
        #     package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #     parameters=parameters,
        #     remappings=remappings),
        # Node(
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=parameters,
        #     remappings=remappings,
        #     arguments=['-d']),
        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=parameters,
        #     remappings=remappings),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
        
        # The IMU frame is missing in TF tree, add it:
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_imu_frame', 'camera_imu_optical_frame']),

        # for sim action
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']),
    ])