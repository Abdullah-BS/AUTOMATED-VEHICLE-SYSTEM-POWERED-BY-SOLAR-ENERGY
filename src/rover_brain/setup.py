import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Rover Brain - GPS Navigation and AI Vision',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'commander = rover_brain.commander:main',
            'lidar_relay = rover_brain.lidar_relay:main', # <-- REGISTERED THE NEW NODE
        ],
    },
)