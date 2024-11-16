from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'basic_pubsub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jegoh',
    maintainer_email='jeffrey.goh@sit.singaporetech.edu.sg',
    description='Basic publisher subscriber package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = basic_pubsub.publisher_node:main',
            'subscriber = basic_pubsub.subscriber_node:main',
        ],
    },
)
