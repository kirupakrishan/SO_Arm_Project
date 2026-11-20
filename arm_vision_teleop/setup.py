import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_vision_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'windows'), glob('windows/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirupa_krishan',
    maintainer_email='kirupa_krishan@todo.todo',
    description='Webcam hand-tracking teleoperation of the SO-101 arm (MediaPipe)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_bridge = arm_vision_teleop.camera_bridge:main',
            'hand_tracker = arm_vision_teleop.hand_tracker:main',
            'hand_follower = arm_vision_teleop.hand_follower:main',
        ],
    },
)
