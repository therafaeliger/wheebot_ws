import os
import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    rtabmap_launch_dir = get_package_share_directory("rtabmap_launch")
    default_db_path = "~/.ros/rtabmap.db"

    # --- 1. Ambil Nilai Argumen ---
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    localization_mode = LaunchConfiguration('localization_mode').perform(context).lower() == 'true'
    use_dor = LaunchConfiguration('use_dor').perform(context).lower() == 'true'
    enable_logging = LaunchConfiguration('enable_logging').perform(context).lower() == 'true'
    delete_db_flag = LaunchConfiguration('delete_db_on_start').perform(context).lower() == 'true'
    
    # --- 2. Konfigurasi Database Path ---
    if localization_mode:
        db_path = default_db_path
    else:
        if enable_logging:
            # Mapping dengan Log Baru (Timestamp) -> Selalu file baru
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            db_dir = "/home/rafael/Documents/dbdb"
            os.makedirs(db_dir, exist_ok=True)
            db_path = os.path.join(db_dir, f"rtabmap_{timestamp}.db")
        else:
            # Mapping overwrite/append default
            db_path = default_db_path

    # --- 3. Konfigurasi RTAB-Map Args ---
    rtabmap_args_list = []

    # PERUBAHAN LOGIKA DI SINI:
    # Hapus DB hanya jika:
    # 1. BUKAN Lokalisasi (Mapping)
    # 2. TIDAK Logging Kustom (artinya pakai default path yang mungkin sudah ada isinya)
    # 3. Flag Delete dinyalakan user
    if delete_db_flag and (not localization_mode and not enable_logging):
        rtabmap_args_list.append("--delete_db_on_start")

    # Grid parameters (Hanya untuk Real Robot)
    if not use_sim_time:
        rtabmap_args_list.append(
            "--Grid/MinGroundHeight -0.4 "
            "--Grid/MaxGroundHeight 0.4 "
            "--Grid/MaxObstacleHeight 1.0 "
            "--Grid/RangeMin 0.1 "
            "--Grid/RangeMax 4.0 "
            "--Grid/NoiseFilteringRadius 0.1 "
            "--Grid/NoiseFilteringMinNeighbors 3 "
            "--Grid/VoxelSize 0.05 "
            "--Grid/FootprintLength 0.5 "
            "--Grid/FootprintWidth 0.4"
        )
    
    final_rtabmap_args = " ".join(rtabmap_args_list)
    print("---------------------------------------------")
    print(f"Database Path: {db_path}") # Debugging path
    print(f"Arguments    : {final_rtabmap_args}")
    print("---------------------------------------------")

    # --- 4. Konfigurasi Topik ---
    rgb_topic = ''
    depth_topic = ''
    camera_info_topic = ''
    imu_topic = ''
    scan_cloud_topic = '/scan_cloud' # Dummy default
    subscribe_scan_cloud = 'false'

    if use_dor:
        rgb_topic = '/dor/inpainted/image'
        depth_topic = '/dor/dynamic_removed/depth'
        # depth_topic = '/camera/depth/image_rect_raw'
        imu_topic = '/camera/imu' if use_sim_time else '/imu/data'
        
        subscribe_scan_cloud = 'false' # Jika DOR aktif, ini biasanya true
        scan_cloud_topic = '/dor/dynamic_removed/pointcloud'

        if not use_sim_time:
            camera_info_topic = '/camera/color/camera_info'
        else:
            camera_info_topic = '/camera/camera_info'
        
    else:
        if use_sim_time:
            rgb_topic = '/camera/image'
            depth_topic = '/camera/depth_image'
            camera_info_topic = '/camera/camera_info'
            imu_topic = '/camera/imu'
        else:
            rgb_topic = '/camera/color/image_raw'
            depth_topic = '/camera/depth/image_rect_raw'
            camera_info_topic = '/camera/color/camera_info'
            imu_topic = '/imu/data'

    # --- 5. Include Launch ---
    rtabmap_launch = IncludeLaunchDescription(
        os.path.join(rtabmap_launch_dir, "launch", "rtabmap.launch.py"),
        launch_arguments={
            'use_sim_time': str(use_sim_time).lower(),
            'args': final_rtabmap_args,
            
            'rtabmap_viz': 'true',
            'rviz': 'true',
            'namespace': '',
            
            # 'qos': '2',

            'localization': str(localization_mode).lower(),
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'map_topic': 'map',
            'publish_tf_map': 'true',
            
            'database_path': db_path,
            
            'queue_size': '10',
            'topic_queue_size': '10',
            'wait_for_transform': '1.0',
            'approx_sync': 'true',
            'approx_rgbd_sync': 'true',
            'approx_sync_max_interval': '0.05',

            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'imu_topic': imu_topic,
            
            'stereo': 'false',
            'subscribe_rgbd': 'false',
            'subscribe_scan': 'false',
            'scan_topic': '/scan',
            'subscribe_scan_cloud': subscribe_scan_cloud,
            'scan_cloud_topic': scan_cloud_topic,

            'visual_odometry': 'true',
            'icp_odometry': 'false',
            'odom_topic': 'odom',
            'publish_tf_odom': 'true',
            'wait_imu_to_init': 'true',
            'always_check_imu_tf': 'false',
            
        }.items()
    )

    return [rtabmap_launch]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization_mode',
            default_value='false',
            description='Launch in localization mode (true) or mapping mode (false)'
        ),
        
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='true',
            description='Delete database on start? (Ignored if localization_mode is true)'
        ),
        
        DeclareLaunchArgument(
            'use_dor',
            default_value='false',
            description='Use Dynamic Object Removal topics'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time and topics'
        ),

        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='If true (and not in localization), save DB to custom path with timestamp'
        ),

        OpaqueFunction(function=launch_setup)
    ])