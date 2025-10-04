from setuptools import setup
from glob import glob
import os

package_name = 'rr_assignment'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),   # <-- add this
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zaka',
    maintainer_email='zaka@example.com',
    description='Reactive robot assignment controllers and launch files',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'host_controller = rr_assignment.host_controller:main',
            'guest_controller = rr_assignment.guest_controller:main',
        ],
    },
)
