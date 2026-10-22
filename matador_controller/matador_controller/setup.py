import os
from glob import glob
from setuptools import setup

package_name = 'matador_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='AprilTag follower controller for Assignment 3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tag_follower = matador_controller.tag_follower:main',
        ],
    },
)
