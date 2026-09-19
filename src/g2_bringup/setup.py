from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'g2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poultry',
    maintainer_email='woodyspivey@gmail.com',
    description='Basic ROS 2 demo package (talker/listener) for the g2-2026 workspace.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'g2_talker = g2_bringup.g2_talker:main',
            'g2_listener = g2_bringup.g2_listener:main',
        ],
    },
)
