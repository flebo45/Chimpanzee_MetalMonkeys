from setuptools import setup
import os
from glob import glob

package_name = 'hunter_control'
submodules = 'hunter_control/behaviors'

setup(
    name=package_name,
    version='0.0.0',
    # Qui diciamo a setuptools di includere il pacchetto principale E i sottomoduli
    packages=[package_name, submodules, submodules+'/actions', submodules+'/conditions'],
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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sintassi: 'nome_comando = nome_cartella.nome_file:main'
            'control_node = hunter_control.hunter_bt_node:main',
        ],
    },
)
