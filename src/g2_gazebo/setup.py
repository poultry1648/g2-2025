from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'g2_gazebo'


def model_data_files():
    data_files = []
    for root, _dirs, files in os.walk('models'):
        if files:
            data_files.append((
                os.path.join('share', package_name, root),
                [os.path.join(root, name) for name in files],
            ))
    return data_files


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
    ] + model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poultry',
    maintainer_email='woodyspivey@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
