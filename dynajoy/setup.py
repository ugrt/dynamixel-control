import os
from setuptools import find_packages, setup

package_name = 'dynajoy'

# Resolve config path relative to this file so colcon can be run from anywhere
_here = os.path.dirname(os.path.abspath(__file__))
_config = os.path.join(_here, '..', 'config', 'robot_params.yaml')

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/config', [_config]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julien',
    maintainer_email='julien.serbanescu@gmail.com',
    description='Dynamixel arm joystick teleop with FK/IK',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_publisher = dynajoy.command_publisher:main'
        ],
    },
)
