from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # parameters_rs=[{
    #       'frame_id':'base_link',
    #       'subscribe_depth':True,
    #       'subscribe_odom_info':True,
    #       'approx_sync':False,
    #       'wait_imu_to_init':True}]

    # remappings_rs=[
    #       ('imu', '/imu/data'),
    #       ('rgb/image', '/dor/inpainted'),
    #       ('rgb/camera_info', '/camera/color/camera_info'),
    #       ('depth/image', '/dor/static_depth')]

    parameters_rp=[{
          'frame_id':'base_link',
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':False}]

    remappings_rp=[
          ('scan', '/scan'),
          ('odom', '/odom'),
          ('imu', '/imu/data')]
    
    return LaunchDescription([

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

        # DeclareLaunchArgument(
        #     'unite_imu_method', default_value='2',
        #     description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        # # Make sure IR emitter is enabled
        # SetParameter(name='depth_module.emitter_enabled', value=1),

        # # Launch camera driver
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

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=parameters_rp,
            remappings=remappings_rp),

        # Node(
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=parameters_rs,
        #     remappings=remappings_rs,
        #     arguments=['-d']),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=parameters_rs,
        #     remappings=remappings_rs),

        # Compute quaternion of the IMU
        # Node(
        #     package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        #     parameters=[{'use_mag': False, 
        #                  'world_frame':'enu', 
        #                  'publish_tf':False}],
        #     remappings=[('imu/data_raw', '/camera/imu')]),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0.63', '0', '0', '0', 'base_link', 'camera_link']
        # ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.7', '3.14159265', '0', '0', 'base_link', 'laser_frame']
        ),
    ])
