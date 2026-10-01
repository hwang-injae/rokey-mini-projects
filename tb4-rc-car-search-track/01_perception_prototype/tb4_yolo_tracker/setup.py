import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tb4_yolo_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Injae Hwang',
    maintainer_email='280895779+hwang-injae@users.noreply.github.com',
    description='YOLO + ByteTrack tracking node for the TurtleBot4 OAK-D image',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_tracker = tb4_yolo_tracker.yolo_tracker_node:main',
        ],
    },
)
