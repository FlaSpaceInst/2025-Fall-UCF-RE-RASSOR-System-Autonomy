import os
from glob import glob
from setuptools import setup

package_name = 're_rassor_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RE-RASSOR Team',
    maintainer_email='re-rassor@ucf.edu',
    description='Gazebo simulation server for RE-RASSOR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_server = re_rassor_simulation.simulation_server:main',
            'nav2_debug = re_rassor_simulation.nav2_debug:main',
        ],
    },
)
