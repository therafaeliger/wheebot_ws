import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    wheebot_description_dir = get_package_share_directory("wheebot_description")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(wheebot_description_dir, "urdf", "wheebot.urdf.xacro"),
        description="Absolute path to robot URDF file",
    )

    robot_description = ParameterValue(Command([
        "xacro ",
        LaunchConfiguration("model"),
        " is_ignition:=", is_ignition
        ]),
        value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(wheebot_description_dir).parent.resolve())
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"
        ]),
        launch_arguments=[
            ("gz_args", [" -v 4", " -r", " empty.sdf"])
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "wheebot",
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        remappings=[
            ("/imu", "/imu/out")
        ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])