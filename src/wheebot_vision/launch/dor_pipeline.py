from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheebot_vision',
            executable='object_detection_and_classification',
            name='object_detection_and_classification',
            output='screen',
            parameters=[{
                'yolo_model': 'yolov8n-seg.pt',
                'dynamic_classes': ['person','car','truck','motorbike','bicycle','bus','dog','cat'],
                'static_classes': ['chair','table','sofa','monitor','tv','bed','refrigerator'],
                'mask_dilate': 5,
                'conf_thres': 0.5,
                'show_debug': True,
            }]
        ),
        Node(
            package='wheebot_vision',
            executable='dynamic_object_removal',
            name='dynamic_object_removal',
            output='screen',
            parameters=[{
                'inpaint_radius': 3,
            }],
        ),
        Node(
            package='wheebot_vision',
            executable='pointcloud_republisher',
            name='pointcloud_republisher',
            output='screen',
        ),
    ])