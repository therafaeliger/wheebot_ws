import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    sensor_run = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_firmware"),
            "launch",
            "sensor.launch.py"
        ),
    )

    lidar_filter = Node(
        package='wheebot_utils',
        executable='lidar_range.py',
        name='lidar_range',
        output='screen',
        parameters=[{
            'ignore_angle_deg': 70.0,
            'ignore_center_deg': 30.0,
            'min_range_ignore': 0.15,
        }]
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

    # imu_driver_node = Node(
    #     package="wheebot_firmware",
    #     executable="mpu6050_driver.py"
    # )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
        # sensor_run,
        # lidar_filter,
        # localization,
        # slam,
        # navigation,
        # imu_driver_node,

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