from setuptools import setup
from glob import glob
import os

package_name = 'ros2vision_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ZHU WEIZHE',
    maintainer_email='your_email@example.com',
    description='System-level bringup package for ROS2vision.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)