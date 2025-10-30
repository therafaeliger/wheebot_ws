import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": "small_house"
        }.items()
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_localization"),
            "launch",
            "rtabmap_fused_odom.launch.py"
        ),
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_mapping"),
            "launch",
            "rtabmap_fused_slam.launch.py"
        ),
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )
    
    return LaunchDescription([
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        navigation,

        # Static TF
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
            arguments=['0.3036', '0.2157', '0.7', '3.1416', '0', '0', 'base_link', 'laser_link']
        ),
    ])