from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'M3Pro_demo'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'apriltag_detect	= M3Pro_demo.apriltag_detect:main',
        'grasp_desktop	= M3Pro_demo.grasp_desktop:main' ,
        'apriltag_list	= M3Pro_demo.apriltag_list:main',
        'mediapipe_detect = M3Pro_demo.mediapipe_detect:main',
        'apriltagID_gesture = M3Pro_demo.apriltagID_gesture:main',
        'apriltagHeight_gesture = M3Pro_demo.apriltagHeight_gesture:main',
        'apriltagDist_gesture = M3Pro_demo.apriltagDist_gesture:main',
        'color_recognize = M3Pro_demo.color_recognize:main',
        'color_list = M3Pro_demo.color_list:main',
        'shape_recognize = M3Pro_demo.shape_recognize:main',
        'volume_calculations = M3Pro_demo.volume_calculations:main',
        'follow_line = M3Pro_demo.follow_line:main',
        'apriltag_track = M3Pro_demo.apriltag_track:main',
        'grasp	= M3Pro_demo.grasp:main' ,
        'Gesture_Moving = M3Pro_demo.Gesture_Moving:main',
        'apriltag_transport = M3Pro_demo.apriltag_transport:main',
        'grasp_transport = M3Pro_demo.grasp_transport:main',
        
        'color_transport = M3Pro_demo.color_transport:main',
        'M3Pro_Dancing = M3Pro_demo.M3Pro_Dancing:main',
        'color_track = M3Pro_demo.color_track:main',
        'KCF_track = M3Pro_demo.KCF_track:main',
        'apriltag_follow = M3Pro_demo.apriltag_follow:main',   
        'color_follow = M3Pro_demo.color_follow:main', 
        'KCF_follow = M3Pro_demo.KCF_follow:main',
        'apriltag_track_desktop = M3Pro_demo.apriltag_track_desktop:main',
        'color_track_desktop = M3Pro_demo.color_track_desktop:main',
        'apriltag_follow_2D = M3Pro_demo.apriltag_follow_2D:main',
        'apriltag_transport_V2 = M3Pro_demo.apriltag_transport_V2:main',
        'KCF_follow_desktop = M3Pro_demo.KCF_follow_desktop:main',
        'estimate_volume = M3Pro_demo.estimate_volume:main',
        'arm_offset = M3Pro_demo.arm_offset:main',
		'mediapipe_gesture = M3Pro_demo.mediapipe_gesture:main',
        'apriltag_rotation = M3Pro_demo.apriltag_rotation:main',
        'grasp_joint5	= M3Pro_demo.grasp_joint5:main' ,
        'test_mediapipe = M3Pro_demo.test_mediapipe:main'
        ],
    },
)
