from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'), glob('action/*.action'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jegoh',
    maintainer_email='jeffrey.goh@sit.singaporetech.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_server = robot_monitor.battery_action_server:main',
            'battery_client = robot_monitor.battery_action_client:main'
        ],
    },
)
