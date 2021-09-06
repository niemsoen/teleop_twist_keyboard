import os
from glob import glob
from setuptools import setup

package_name = 'teleop_twist_keyboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sönke Niemann',
    maintainer_email='soenke.niemann@ipk.fraunhofer.de',
    description='A fork of the teleop_twist_keyboard',
    license='BSD License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_twist_keyboard = teleop_twist_keyboard.teleop_twist_keyboard_node:main',
        ],
    },
)
