import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Deklarasi Argumen ---
    
    # Argumen Waktu Simulasi
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Argumen SLAM / RTAB-Map
    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='false', # Sesuai request: default true (lokalisasi)
        description='Launch RTAB-Map in localization mode'
    )
    
    use_dor_arg = DeclareLaunchArgument(
        'use_dor',
        default_value='true', # Sesuai request: default true
        description='Use Dynamic Object Removal topics'
    )
    
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='false', # Sesuai request: default false
        description='Enable logging to custom DB path'
    )
    
    delete_db_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='true', # Sesuai request: default true
        description='Delete database on start (ignored if localization is true)'
    )

    # --- 2. Konfigurasi Variabel ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_mode = LaunchConfiguration('localization_mode')
    use_dor = LaunchConfiguration('use_dor')
    enable_logging = LaunchConfiguration('enable_logging')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')

    # --- 3. Include Launch Files ---

    # A. Run Hardware
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wheebot_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        )
    )
    sensor_run = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wheebot_firmware"),
                "launch",
                "sensor.launch.py"
            )
        )
    )
    delayed_sensor_run = TimerAction(
        period=5.0,
        actions=[sensor_run]
    )

    
    # B. Run Controller
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wheebot_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )

    # C. Run Teleop dan Twist Mux
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wheebot_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    # D. Dynamic Object Removal Pipeline (DOR)
    dor_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wheebot_vision"),
                "launch",
                "dor_pipeline.launch.py" 
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    # E. RTAB-Map SLAM / Localization (BARU DITAMBAHKAN)
    rtabmap_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("wheebot_mapping"),
                "launch",
                "rtabmap_slam.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "localization_mode": localization_mode,
            "use_dor": use_dor,
            "enable_logging": enable_logging,
            "delete_db_on_start": delete_db_on_start
        }.items()
    )

    delayed_rtabmap = TimerAction(
        period=10.0,
        actions=[rtabmap_slam]
    )

    # F Navigasi
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("wheebot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    delayed_navigation = TimerAction(
        period=15.0,
        actions=[navigation]
    )

    # Static TF
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.3616', '0.2157', '0.63', '-0.1', '0', '0', 'base_link', 'camera_link']
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        localization_mode_arg,
        use_dor_arg,
        enable_logging_arg,
        delete_db_arg,
        
        # Nodes
        # hardware_interface,
        # delayed_sensor_run,
        # controller,
        # joystick,
        dor_pipeline,
        delayed_rtabmap,
        # delayed_navigation,

        # tf (if needed)
        static_tf_camera,
    ])