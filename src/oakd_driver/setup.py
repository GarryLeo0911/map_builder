from setuptools import setup
import os
from glob import glob

package_name = 'oakd_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/oakd_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/oakd_params.yaml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 driver for OAK-D camera providing point clouds, RGB images, and depth data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oakd_node = oakd_driver.oakd_node:main',
            'oakd_publisher = oakd_driver.oakd_publisher:main',
        ],
    },
)