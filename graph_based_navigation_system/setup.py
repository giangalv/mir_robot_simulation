import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'graph_based_navigation_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mir250ur5e',
    maintainer_email='gianluca.galvagni@edu.unige.it',
    description='Graph-based Navigation System for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_editor = graph_based_navigation_system.graph_editor:main',
            'render_graph_on_map = graph_based_navigation_system.render_graph_on_map:main',
            'graph_nav_controller = graph_based_navigation_system.graph_nav_controller:main',
        ],
    },
)
