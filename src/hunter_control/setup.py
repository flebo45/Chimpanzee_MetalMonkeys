from setuptools import setup
import os
from glob import glob

package_name = 'hunter_control'
submodules = 'hunter_control/behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.behaviors'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Hunter Control Package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'control_node = hunter_control.bt_main:main',
            'ball_teleop = hunter_control.ball_teleop:main',
        ],
    },
)
