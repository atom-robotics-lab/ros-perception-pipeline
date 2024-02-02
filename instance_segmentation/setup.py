from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'instance_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'vision_msgs'],
    zip_safe=True,
    maintainer='deepansh',
    maintainer_email='gl.deepansh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'InstanceSegmentation = instance_segmentation.InstanceSegmentation:main',
        ],
    },
)
