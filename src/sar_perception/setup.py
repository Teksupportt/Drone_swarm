from setuptools import setup
import os
from glob import glob

package_name = 'sar_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'drone_perception_node = sar_perception.drone_perception_node:main',
        ],
    },
)
