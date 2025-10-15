# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    parameters=[{
          'use_sim_time': True,
          'frame_id':'base_link',
          'subscribe_scan': True,
          'subscribe_scan_cloud': False,
          'subscribe_depth': False,
          'subscribe_rgb': False,
          'subscribe_odom_info': True,
          'approx_sync':False,
          'wait_imu_to_init':True}]

    remappings=[
          ('imu', '/imu/out'),
          ('scan', '/scan')]

    return LaunchDescription([

        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     name='rplidar_a1',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'serial_baudrate': 115200,
        #         'frame_id': 'laser_frame',
        #         'inverted': False,
        #         'angle_compensate': True
        #     }]
        # ),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.7', '3.14159265', '0', '0', 'base_link', 'laser_link']
        ),
    ])
