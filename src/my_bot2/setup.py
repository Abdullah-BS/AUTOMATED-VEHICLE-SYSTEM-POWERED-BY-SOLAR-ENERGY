from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_bot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',  glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',  glob('config/*.yaml')),
        ('share/' + package_name + '/urdf',    glob('urdf/*')),
        ('share/' + package_name + '/worlds',  glob('worlds/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/meshes',  glob('meshes/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='AhmedAlharthy017@gmail.com',
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
