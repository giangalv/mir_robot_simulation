import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'manual_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mir250ur5e',
    maintainer_email='gianluca.galvagni@edu.unige.it',
    description='Manual Navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_mode = manual_navigation.manual_mode:main',
            'cloud_transformation = manual_navigation.cloud_transformation:main',
            'encoder_to_joint_state = manual_navigation.encoder_to_joint_state:main'
        ],
    },
)
