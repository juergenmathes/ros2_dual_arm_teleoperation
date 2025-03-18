from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'teleop_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Mark this package as a ROS package
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
         
        # Include our package.xml
        ('share/' + package_name, ['package.xml']),
        
        # Install all the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install config files, URDF models
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Dual robot arm teleoperation using vision-based hand tracking',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_publisher_node = teleop_arm_control.joint_publisher_node:main',
            'teleoperation_node = teleop_arm_control.teleoperation_node:main',
        ],
    },
)