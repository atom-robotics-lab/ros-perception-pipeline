from glob import glob
import os

from setuptools import setup

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'test'), glob('test/*.py')),
        (os.path.join('share', package_name, 'test/test_bag'), glob('test/test_bag/*')),
        (os.path.join('share', package_name, 'test/test_config'), glob('test/test_config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='singh',
    maintainer_email='jasmeet0915@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ObjectDetection = object_detection.ObjectDetection:main',
        ],
    },
)
