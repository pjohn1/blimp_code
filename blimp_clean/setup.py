from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'blimp_clean'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c3',
    maintainer_email='P_J_Johnson@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'setup_gui_node = blimp_clean.setup_gui_node:main',
            'serial_node = blimp_clean.serial_node:main',
            'optitrack_node = blimp_clean.optitrack_node:main',
            'teleop_receiver = blimp_clean.teleop_receiver:main',
        ],
    },
)
