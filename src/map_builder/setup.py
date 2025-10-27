from setuptools import find_packages, setup

package_name = 'map_builder'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/map_builder.launch.py',
            '../../launch/complete_system.launch.py',
            '../../launch/laptop_side.launch.py',
            '../../launch/robot_side.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/map_builder_params.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/map_builder.rviz']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'scikit-learn',
        'open3d',
        'matplotlib',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='3D mapping and surface reconstruction package for OAK-D camera data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_cloud_processor = map_builder.point_cloud_processor:main',
            'surface_reconstructor = map_builder.surface_reconstructor:main',
            'map_builder_node = map_builder.map_builder_node:main',
        ],
    },
)