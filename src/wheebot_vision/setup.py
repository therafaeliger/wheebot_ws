from setuptools import find_packages, setup

package_name = 'wheebot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafael',
    maintainer_email='rafaeliger.hadikusuma@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_sub = wheebot_vision.camera_sub:main',
            'camera_pub = wheebot_vision.camera_pub:main',
            
            'object_detection = wheebot_vision.object_detection:main',
            'object_classification = wheebot_vision.object_classification:main',
            'dynamic_object_removal = wheebot_vision.dynamic_object_removal:main',
            'depth_republisher = wheebot_vision.depth_republisher:main',

            'dynamic_object_removal_full_system = wheebot_vision.dynamic_object_removal_full_system:main',
            'dynamic_object_removal_sync = wheebot_vision.dynamic_object_removal_sync:main',

            # 'depth_dor = wheebot_vision.depth_dor:main',
            # 'dor = wheebot_vision.dor:main',
            # 'inpainting = wheebot_vision.inpainting:main',
            # 'obj_class = wheebot_vision.obj_class:main',
            # 'obj_det = wheebot_vision.obj_det:main',
            # 'obj_seg = wheebot_vision.obj_seg:main',
            # 'yolo_node = wheebot_vision.yolo_node:main',
        ],
    },
)
