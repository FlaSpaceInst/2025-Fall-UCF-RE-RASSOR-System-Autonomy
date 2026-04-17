from setuptools import find_packages, setup

package_name = 're_rassor_computer_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RE-RASSOR Team',
    maintainer_email='your.email@ucf.edu',
    description='ArUco marker detection node for RE-RASSOR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_detector = re_rassor_computer_vision.aruco_detector_node:main',
        ],
    },
)
