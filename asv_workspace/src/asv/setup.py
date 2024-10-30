from setuptools import setup
import os
from glob import glob

package_name = 'asv'
submodule = 'asv/submodulos'

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
    maintainer='Luis Miguel',
    maintainer_email='luismidp7@gmail.com',
    description='Autonomous Surface Vehicles for Environmental Monitoring',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wqp_sensor = asv.wqp_sensor_node:main',
            'sonar = asv.sonar_node:main',
            'asv = asv.asv_node:main',
            'communications = asv.server_communication_node:main',
            'path_planner = asv.path_planner_node:main',
            'database = asv.database_node:main',
        ],
    },
)
