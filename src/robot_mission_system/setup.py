from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_mission_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/data', glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Ops',
    maintainer_email='ops@local',
    description='Waypoint teach + Nav2 mission orchestration for differential robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teach_waypoint = robot_mission_system.teach_waypoint:main',
            'mission_manager = robot_mission_system.mission_manager:main',
            'system_status = robot_mission_system.system_status:main',
            'operator_cli = robot_mission_system.operator_cli:main',
        ],
    },
)
