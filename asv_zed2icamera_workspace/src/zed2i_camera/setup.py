import os
from glob import glob
from setuptools import setup

package_name = 'zed2i_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, "urdf"), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, "config"), glob('config/*.yaml')),
        (os.path.join('share',  package_name, "utils", "xavier_weights"), glob(os.path.join('zed2i_camera','utils','weights/*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Miguel',
    maintainer_email='luismidp7@gmail.com',
    description='Autonomous Surface Vehicles for Object Detection in Water',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_object_detection = zed2i_camera.custom_object_detection:main',
            'test_node = zed2i_camera.test_node:main'
        ],
    },
)
