from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotx'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # Automatically find and include all packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugues',
    maintainer_email='hchardin3@gatech.edu',
    description='A ROS 2 package for autonomous drone operations',  # TODO: Fill in a real description
    license='Apache-2.0',  # TODO: Choose an appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_node = robotx.drone_node:main',
            'camera_node = robotx.camera_node:main',
            'mission_node = robotx.mission_node:main',
        ],
    },
)
