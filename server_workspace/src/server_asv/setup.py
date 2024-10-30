from setuptools import setup
import os
from glob import glob

package_name = 'server_asv'
submodule = 'server_asv/submodulos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alejandro Casado',
    maintainer_email='acasado4@us.es',
    description='Server Side for Autonomous Surface Vehicles for Environmental Monitoring - US/Loyola',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'state_composer = server_asv.state_composer_node:main',
            'logger = server_asv.logger_node:main',
            'algorithm = server_asv.algorithm_node:main',
            'ship_communications = server_asv.ship_communications_node:main',
        ],
    },
)
