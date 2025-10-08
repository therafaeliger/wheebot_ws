#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'yolo_model',
            default_value='yolo11s-seg.pt',
            description='YOLO model file for segmentation'
        ),
        DeclareLaunchArgument(
            'show_debug',
            default_value='true',
            description='Show debug visualization windows'
        ),
        
        Node(
            package='wheebot_vision',
            executable='dynamic_object_removal_full_system',
            name='dynamic_object_removal_full_system',
            output='screen',
            parameters=[{
                'yolo_model': LaunchConfiguration('yolo_model'),
                'show_debug': LaunchConfiguration('show_debug'),
                'dynamic_classes': ['person', 'car', 'truck', 'motorbike', 'bicycle', 'bus', 'dog', 'cat'],
                'static_classes': ['chair', 'table', 'sofa', 'monitor', 'tv', 'bed', 'refrigerator'],
                'inpaint_radius': 3,
                'mask_dilate': 2
            }]
        )
    ])