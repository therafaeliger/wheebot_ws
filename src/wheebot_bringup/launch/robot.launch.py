import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

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

    # imu_driver_node = Node(
    #     package="wheebot_firmware",
    #     executable="mpu6050_driver.py"
    # )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )
    
    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        controller,
        joystick,
        # imu_driver_node,
        localization,
        slam,
        navigation
    ])