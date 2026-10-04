import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_processing'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harsh-mahamuni',
    maintainer_email='harsh@todo.todo',
    description='Camera processing and calibration publishing nodes for SAR quadcopter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'image_decoder = camera_processing.image_decoder:main',
            'camera_info_publisher = camera_processing.camera_info_publisher:main',
        ],
    },
)
