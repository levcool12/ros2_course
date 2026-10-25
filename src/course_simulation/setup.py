import os
from glob import glob
from setuptools import setup

package_name = 'course_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/contracts', glob('contracts/*.md')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models/course_ackermann',
            glob('models/course_ackermann/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lev',
    maintainer_email='lev@example.com',
    description='PR07: headless Gazebo simulation with drive/odometry adapters.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_adapter = course_simulation.drive_adapter:main',
            'odom_adapter = course_simulation.odom_adapter:main',
        ],
    },
)
