from setuptools import setup
import os
from glob import glob

package_name = 'dexi_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dennis Baldwin',
    maintainer_email='db@droneblocks.io',
    description='DEXI tools package for firmware flashing and system utilities',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'firmware_flash_node = dexi_tools.firmware_flash_node:main',
        ],
    },
)