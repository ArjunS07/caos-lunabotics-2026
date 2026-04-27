from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lunabotics_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lunabotics Team',
    maintainer_email='team@lunabotics.local',
    description='Crater rim detection using RealSense D435i depth image.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'crater_detector = lunabotics_detection.crater_detector_node:main',
            'crater_visualizer = lunabotics_detection.crater_visualizer_node:main',
        ],
    },
)
