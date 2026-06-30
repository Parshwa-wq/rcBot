from setuptools import setup
import os
from glob import glob

package_name = 'ocpkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Install marker file for package discovery
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Mecanum drive robot with PID control and odometry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps5_teleop_node = ocpkg.ps5_teleop_node:main',
            'pid_controller_node = ocpkg.pid_controller_node:main',
        ],
    },
)

