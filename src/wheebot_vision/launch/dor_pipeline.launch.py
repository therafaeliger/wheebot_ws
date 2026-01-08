from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    
    if use_sim_time == 'true':
        # --- KONFIGURASI SIMULASI (GAZEBO/ISAAC) ---
        print("--> MODE: SIMULASI")
        camera_info_topic  = '/camera/camera_info'
        camera_image_topic = '/camera/image'
        camera_depth_topic = '/camera/depth_image'
    else:
        # --- KONFIGURASI REAL ROBOT (REALSENSE) ---
        print("--> MODE: REAL ROBOT")
        camera_info_topic  = '/camera/color/camera_info'
        camera_image_topic = '/camera/color/image_raw'
        camera_depth_topic = '/camera/aligned_depth_to_color/image_raw'

    return [
        Node(
            package='wheebot_vision',
            executable='object_detection_and_classification',
            name='object_detection_and_classification',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yolo_model': 'yolov8n-seg.pt',
                'dynamic_classes': ['person','car','truck','motorbike','bicycle','bus','dog','cat'],
                'static_classes': ['chair','table','sofa','monitor','tv','bed','refrigerator'],
                'mask_dilate': 5,
                'conf_thres': 0.5,
                'show_debug': True,
                # Gunakan variabel
                'camera_info_topic': camera_info_topic,
                'camera_image_topic': camera_image_topic,
            }]
        ),
        Node(
            package='wheebot_vision',
            executable='dynamic_object_removal',
            name='dynamic_object_removal',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'inpaint_radius': 3,
                'mask_topic': '/yolo/mask/dynamic_only',
                # Gunakan variabel
                'camera_info_topic': camera_info_topic,
                'camera_image_topic': camera_image_topic,
                'camera_depth_topic': camera_depth_topic,
            }],
        ),
        Node(
            package='wheebot_vision',
            executable='pointcloud_republisher',
            name='pointcloud_republisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mask_dynamic_topic': '/yolo/mask/dynamic_only',
                'mask_static_topic': '/yolo/mask/static_only',
                # Gunakan variabel
                'camera_info_topic': camera_info_topic,
                'camera_depth_topic': camera_depth_topic,
            }],
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gunakan true untuk simulasi, false untuk real robot'
        ),
        
        # Panggil fungsi setup
        OpaqueFunction(function=launch_setup)
    ])