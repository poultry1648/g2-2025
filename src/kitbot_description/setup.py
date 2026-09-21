from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'kitbot_description'


def data_files_from(path):
    data_files = []
    for root, _dirs, files in os.walk(path):
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
    ] + data_files_from('urdf') + data_files_from('meshes'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poultry',
    maintainer_email='woodyspivey@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
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
