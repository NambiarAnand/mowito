from setuptools import setup
import os
from glob import glob

package_name = 'camera_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resources'), glob('resources/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Camera image processing package with dummy USB cam and image conversion',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_usb_cam = camera_processing.dummy_usb_cam_node:main',
            'image_conversion = camera_processing.image_conversion_node:main',
        ],
    },
)