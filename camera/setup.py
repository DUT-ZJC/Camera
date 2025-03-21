from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DUT-ZJC',
    maintainer_email='2563072882@qq.com',
    description='for camera to pulish Extrinsics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_node = camera.camera_node:main',
        'came_node_listener = camera.came_node_listener:main',
        'camera_calibration_node = camera.camera_calibration_node:main',
        'Extrinsics_publish_node = camera.Extrinsics_publish_node:main'
        ],
    },
)
