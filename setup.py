from setuptools import setup
import os
from glob import glob

package_name = 'map_builder'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.nodes', f'{package_name}.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'scikit-learn',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='3D mapping and surface reconstruction package for OAK-D camera data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_cloud_processor = map_builder.nodes.point_cloud_processor:main',
            'surface_reconstructor = map_builder.nodes.surface_reconstructor:main',
            'map_builder_node = map_builder.nodes.map_builder_node:main',
            'system_monitor = map_builder.scripts.system_monitor:main',
            'diagnostics = map_builder.scripts.diagnostics:main',
        ],
    },
)