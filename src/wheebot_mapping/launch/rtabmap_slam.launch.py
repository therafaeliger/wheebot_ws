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

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    localization_mode = LaunchConfiguration('localization_mode').perform(context).lower() == 'true'
    use_dor = LaunchConfiguration('use_dor').perform(context).lower() == 'true'
    enable_logging = LaunchConfiguration('enable_logging').perform(context).lower() == 'true'
    delete_db_flag = LaunchConfiguration('delete_db_on_start').perform(context).lower() == 'true'
    
    if localization_mode:
        db_path = default_db_path
    else:
        if enable_logging:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            db_dir = os.path.expanduser("~/Documents/dbdb")
            os.makedirs(db_dir, exist_ok=True)
            db_path = os.path.join(db_dir, f"rtabmap_{timestamp}.db")
        else:
            db_path = default_db_path

    rtabmap_args_list = []

    if delete_db_flag and (not localization_mode and not enable_logging):
        rtabmap_args_list.append("--delete_db_on_start")

    rtabmap_args_list.append(
        # --- RATE LIMITING ---
        "--Rtabmap/DetectionRate 2 "      # Hanya update map/loop closure 2Hz (Hemat CPU)
        
        # --- FEATURE EXTRACTION (Visual Odometry) ---
        "--Vis/FeatureType 6 "            # 6=GFTT (Paling ringan/cepat), Default=SURF/SIFT (Berat)
        "--Kp/MaxFeatures 400 "           # Batasi jumlah fitur visual (Default 1000 terlalu banyak)
        "--Mem/ImagePreDecimation 2 "     # Kecilkan gambar (half-size) sebelum proses fitur (Boost 4x speed)
        
        # --- MEMORY MANAGEMENT ---
        "--Mem/RehearsalSimilarity 0.3 "  # Toleransi kemiripan untuk keyframe baru
        "--RGBD/LinearUpdate 0.1 "        # Update map hanya jika robot gerak 10cm
        "--RGBD/AngularUpdate 0.1 "       # Update map hanya jika robot putar 0.1 rad
        
        # --- GRID MAP ---
        "--Grid/RangeMin 0.1 "
        "--Grid/RangeMax 4.0 "
        "--Grid/VoxelSize 0.05 "
        # "--Grid/RayTracing false "
    )

    # Grid heights (Sim vs Real)
    if not use_sim_time:
        rtabmap_args_list.append(
            "--Grid/MinGroundHeight -0.4 "
            "--Grid/MaxGroundHeight 0.4 " # 0.1
            "--Grid/MaxObstacleHeight 1.0 " # 1.5

            "--Grid/NoiseFilteringRadius 0.1 "
            "--Grid/NoiseFilteringMinNeighbors 3 "
            "--Grid/FootprintLength 0.5 "
            "--Grid/FootprintWidth 0.4"
        )
    
    final_rtabmap_args = " ".join(rtabmap_args_list)
    
    print("---------------------------------------------")
    print(f"Mode         : {'Localization' if localization_mode else 'Mapping'}")
    print(f"Use DOR      : {use_dor}")
    print(f"Database     : {db_path}")
    print(f"Args         : {final_rtabmap_args}")
    print("---------------------------------------------")

    # Default variables
    rgb_topic = ''
    depth_topic = ''
    camera_info_topic = ''
    imu_topic = ''
    scan_cloud_topic = '/scan_cloud'
    subscribe_scan_cloud = 'false'
    
    use_visual_odometry = 'true' 

    if use_dor:
        rgb_topic = '/dor/dynamic_removed/image' 
        depth_topic = '/dor/dynamic_removed/depth'
        
        scan_cloud_topic = '/dor/dynamic_removed/pointcloud'
        subscribe_scan_cloud = 'false'

        imu_topic = '/camera/imu' if use_sim_time else '/imu/data'
        
        if not use_sim_time:
            camera_info_topic = '/camera/color/camera_info'
        else:
            camera_info_topic = '/camera/camera_info'
        
    else:
        # Mode Tanpa DOR (Standard)
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


    rtabmap_launch = IncludeLaunchDescription(
        os.path.join(rtabmap_launch_dir, "launch", "rtabmap.launch.py"),
        launch_arguments={
            'use_sim_time': str(use_sim_time).lower(),
            'args': final_rtabmap_args,
            
            'rtabmap_viz': 'false', # GUI berat
            'rviz': 'false',
            'namespace': '',
            
            'localization': str(localization_mode).lower(),
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            
            'database_path': db_path,
            
            'queue_size': '30',
            'topic_queue_size': '30',
            
            'approx_sync': 'true',
            'approx_rgbd_sync': 'true',
            
            'approx_sync_max_interval': '0.2', 

            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'imu_topic': imu_topic,
            
            'stereo': 'false',
            'subscribe_rgbd': 'false',
            'subscribe_scan': 'false',
            'subscribe_scan_cloud': subscribe_scan_cloud,
            'scan_cloud_topic': scan_cloud_topic,

            'visual_odometry': use_visual_odometry, 
            'icp_odometry': 'false',
            'odom_topic': 'odom',
            'publish_tf_odom': 'true',
            
            'wait_imu_to_init': 'true',
            'qos': '1',
            
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
            description='Delete database on start?'
        ),
        DeclareLaunchArgument(
            'use_dor',
            default_value='false',
            description='Use Dynamic Object Removal topics'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'enable_logging',
            default_value='true',
            description='Save DB to timestamped path'
        ),
        OpaqueFunction(function=launch_setup)
    ])