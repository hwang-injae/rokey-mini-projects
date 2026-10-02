import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rc_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Injae Hwang',
    maintainer_email='280895779+hwang-injae@users.noreply.github.com',
    description='Webcam-triggered RC-car search & track mission for TurtleBot4',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'webcam_detector = rc_tracker.webcam_detector_node:main',
            'mission_trigger = rc_tracker.mission_trigger_node:main',
            'search_and_track_server = rc_tracker.search_and_track_server:main',
        ],
    },
)
