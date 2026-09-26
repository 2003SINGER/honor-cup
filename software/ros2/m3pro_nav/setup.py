from setuptools import find_packages, setup

package_name = 'm3pro_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/m3pro.launch.py']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'driver_probe = m3pro_nav.driver_probe:main',
            'control_probe = m3pro_nav.control_probe:main',
        ],
    },
)
