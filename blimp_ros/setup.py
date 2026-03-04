from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'blimp_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c3',
    maintainer_email='c3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'optitrack_node = blimp_ros.optitrack_node:main',
            'low_level_controller = blimp_ros.low_level_controller:main',
            'serial_node = blimp_ros.serial_node:main',
            'high_level_controller = blimp_ros.high_level_controller:main',
            'cbf = blimp_ros.cbf:main',
            'teleop_node = blimp_ros.teleop_node:main'
        ],
    },
)
