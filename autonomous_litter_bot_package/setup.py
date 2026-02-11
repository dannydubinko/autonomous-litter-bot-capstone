from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_litter_bot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml', 'autonomous_litter_bot_package/detection_pb2.py']),

        # --- FIX 1: Point to the correct source location for models ---
        # Your models are inside the inner folder 'autonomous_litter_bot_package/models'
        (os.path.join('share', package_name, 'models/segmentation_small_openvino_model'),
         glob('autonomous_litter_bot_package/models/segmentation_small_openvino_model/*')),

        (os.path.join('share', package_name, 'models/segmentation_nano_openvino_model'),
         glob('autonomous_litter_bot_package/models/segmentation_nano_openvino_model/*')),
        
        # --- FIX 2: Install your config files ---
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi5',
    maintainer_email='daniel.dubinko@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # executable_name = package_folder.python_filename:function_name
            'trash_detection_node = autonomous_litter_bot_package.trash_detection_node:main',
            'proto_sender_node = autonomous_litter_bot_package.proto_sender_node:main',
        ],
    },
)