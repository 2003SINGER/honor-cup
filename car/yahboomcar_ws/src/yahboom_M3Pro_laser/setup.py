from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'yahboom_M3Pro_laser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.py*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'laser_Avoidance = yahboom_M3Pro_laser.laser_Avoidance:main',
        'laser_Warning = yahboom_M3Pro_laser.laser_Warning:main',
        'laser_Tracker = yahboom_M3Pro_laser.laser_Tracker:main'
        ],
    },
)
