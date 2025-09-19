from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dexi_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
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
            'px_uploader = dexi_tools.px_uploader:main',
            'reset_fmu_wait_bl = dexi_tools.reset_fmu_wait_bl:main',
        ],
    },
    scripts=[
        'scripts/flash_px4.sh',
    ],
)