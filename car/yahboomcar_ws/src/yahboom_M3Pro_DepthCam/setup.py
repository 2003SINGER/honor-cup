from setuptools import find_packages, setup

package_name = 'yahboom_M3Pro_DepthCam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'edge_detection = yahboom_M3Pro_DepthCam.Edge_Detection:main',
        'GetDepthColor = yahboom_M3Pro_DepthCam.GetDepthColor:main',
        'Measure_Distance = yahboom_M3Pro_DepthCam.Measure_Distance:main'
        ],
    },
)
