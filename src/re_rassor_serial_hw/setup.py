from setuptools import find_packages, setup

package_name = 're_rassor_serial_hw'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='RE-RASSOR Team',
    maintainer_email='your.email@ucf.edu',
    description='Serial hardware driver for RE-RASSOR Arduino boards',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_driver = re_rassor_serial_hw.serial_driver:main',
        ],
    },
)
