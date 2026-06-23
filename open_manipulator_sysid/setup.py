from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'open_manipulator_sysid'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='OpenMANIPULATOR-X system identification package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'excitation_trajectory_print = '
            'open_manipulator_sysid.excitation_trajectory:main',
            'sysid_regress_synthetic = '
            'open_manipulator_sysid.regress:main',
            'sysid_regress_bag = '
            'open_manipulator_sysid.regress:main_bag',
            'bag_to_dataset = '
            'open_manipulator_sysid.bag_to_dataset:main',
            'excitation_runner = '
            'open_manipulator_sysid.excitation_runner:main',
        ],
    },
)
