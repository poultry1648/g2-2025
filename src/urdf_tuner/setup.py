from glob import glob

from setuptools import find_packages, setup

package_name = 'g2_urdf_tuner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poultry',
    maintainer_email='woodyspivey@gmail.com',
    description='Interactive RViz2 editor for URDF joint origins and axes.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'urdf_tuner_gui = g2_urdf_tuner.urdf_tuner_gui:main',
        ],
    },
)
