from setuptools import setup
import os
from glob import glob

package_name = 'people_follower'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotester1',
    maintainer_email='your_email@example.com',
    description='RealSense-based people following for legged robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'people_follower_node = people_follower.people_follower_node:main',
            'people_follower_yolo_node = people_follower.people_follower_yolo:main',
            'posture_mimic_node = people_follower.posture_mimic_node:main',
            'posture_mimic_mediapipe_node = people_follower.posture_mimic_mediapipe_node:main',
        ],
    },
)

