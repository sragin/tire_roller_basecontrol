from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'rollertire_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, glob('resource/*.dbc')),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='koceti',
    maintainer_email='jpkim@koceti.re.kr',
    description='Tire Roller Controller',
    # license='koceti',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_authority = rollertire_control.control_authority:main',
            'can_parser = rollertire_control.can_parser:main',
            # 'can_sender = rollertire_control.can_sender:main',
            # 'io_controller = rollertire_control.io_controller:main',
            # 'bucket_controller = rollertire_control.bucket_controller:main',
            # 'drive_controller = rollertire_control.drive_controller:main',
            # 'navigator = rollertire_control.navigator:main',
        ],
    },
)
