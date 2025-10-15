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
          'odom_frame_id': 'odom',
          'map_frame_id': 'map',
          'odom_topic': '/odometry/filtered',

          'subscribe_rgb':True,
          'subscribe_depth':True,
          'subscribe_odom':True,
          'subscribe_odom_info':True,
          'subscribe_imu':False,

          'approx_sync':True,
          'wait_imu_to_init':False,
          'wait_for_transform':0.5,
          'publish_tf':True,
          'queue_size':30,
    }]

    remappings=[
          # ('imu', '/imu/data'),
          ('rgb/image', '/camera/image'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth_image'),
          # ('odom', '/odometry/filtered'),
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
