import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_receiver'

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
    description='TCP Frame Receiver node for streaming RGB video from Raspberry Pi 5',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tcp_frame_receiver = camera_receiver.tcp_frame_receiver:main',
        ],
    },
)
