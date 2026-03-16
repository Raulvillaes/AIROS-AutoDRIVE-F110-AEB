from setuptools import setup
import os
from glob import glob

package_name = 'aeb_f110'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altanerx',
    maintainer_email='altanerx@todo.todo',
    description='Automatic Emergency Braking (AEB) for F1TENTH on AutoDRIVE using iTTC',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aeb_node = aeb_f110.aeb_node:main',
        ],
    },
)
