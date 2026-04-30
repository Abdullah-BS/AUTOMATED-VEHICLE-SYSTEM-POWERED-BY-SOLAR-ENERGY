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
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools','pyserial'],
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
            'lidar_node = my_bot2.lidar_processor:main',
            'camera_node = my_bot2.camera_processor:main',
            'master_brake = my_bot2.master_brake:main',
            'camera_test = my_bot2.camera_motor_test:main',
            'gui_node = my_bot2.gui_node:main', # Add this line!
            'scan_cleaner = my_bot2.scan_cleaner:main',
            'cmd_vel_to_rc = my_bot2.cmd_vel_to_rc:main',
            'ros_to_pix_mavros = my_bot2.ros_to_pix_mavros:main',
            'ArduinoBridge = my_bot2.ArduinoBridge:main',



        ],
    },
)
