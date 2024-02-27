import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'interbotix_xsarm_dual_andy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch'))), 
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*yaml'))), 
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Nyu',
    maintainer_email='andy.nyu@qq.com',
    description='The interbotix_xsarm_dual package by Andy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
