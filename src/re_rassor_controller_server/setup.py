from setuptools import find_packages, setup

package_name = 're_rassor_controller_server'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'flask', 'flask-cors'],
    zip_safe=True,
    maintainer='RE-RASSOR Team',
    maintainer_email='your.email@ucf.edu',
    description='HTTP controller server for RE-RASSOR with navigate/calibrate endpoints',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_server = re_rassor_controller_server.controller_server:main',
        ],
    },
)
