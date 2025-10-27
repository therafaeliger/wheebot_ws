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
            'full_system = wheebot_vision.full_system:main',
            'object_detection_and_classification = wheebot_vision.object_detection_and_classification:main',
            'dynamic_object_removal = wheebot_vision.dynamic_object_removal:main',
            'pointcloud_republisher = wheebot_vision.pointcloud_republisher:main',
        ],
    },
)
