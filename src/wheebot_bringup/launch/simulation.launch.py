import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # run gazebo simulator using world
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
    
    # run controller
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items(),
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

    # mode 1: using fused odometry
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_localization"),
            "launch",
            "rtabmap_fused_odom.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )
    mapping = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_mapping"),
            "launch",
            "rtabmap_fused_slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    # mode 2: using rtabmap default launch
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_mapping"),
            "launch",
            "rtabmap_slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    # launch the nav2
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
    )

    # Static TF
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.3616', '0.2157', '0.63', '0', '0', '0', 'base_link', 'camera_link']
    )
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.3036', '0.2157', '0.7', '3.1416', '0', '0', 'base_link', 'laser_link']
    )
    
    return LaunchDescription([
        gazebo,

        controller,
        joystick,

        # mode 1: using fused odometry
        localization,
        mapping,

        # mode 2: using rtabmap default launch
        # slam,

        navigation,

        static_tf_camera,
        static_tf_lidar,
    ])