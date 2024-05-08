from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'unstatic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugues',
    maintainer_email='hchardin3@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_unstatic = unstatic.camera_unstatic:main',
            'drone_unstatic = unstatic.drone_unstatic:main',
            'mission_unstatic = unstatic.mission_unstatic:main',
        ],
    },
)
