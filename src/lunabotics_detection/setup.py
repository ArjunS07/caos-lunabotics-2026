from setuptools import setup
import os
from glob import glob

package_name = 'lunabotics_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caos',
    maintainer_email='caos@lunabotics.local',
    description='Crater detection from RealSense depth for Lunabotics rover',
    license='TODO',
    entry_points={
        'console_scripts': [
            'crater_detector = lunabotics_detection.crater_detector_node:main',
            'crater_cloud_pub = lunabotics_detection.crater_cloud_publisher_node:main',
        ],
    },
)
