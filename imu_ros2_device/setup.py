from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'imu_ros2_device'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DrZeroK',
    maintainer_email='DrZeroK@users.noreply.github.com',
    description='Python reference driver for the Yahboom 10-axis IMU (WitMotion protocol)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wit_imu_driver = imu_ros2_device.wit_imu_driver:main',
        ],
    },
)
