from glob import glob
import os
from pathlib import Path

from setuptools import find_packages
from setuptools import setup

package_name = 'open_manipulator_sysid'

_config_dir = Path('config')
_config_files = [str(path) for path in _config_dir.iterdir() if path.is_file()] if _config_dir.is_dir() else []

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), _config_files),
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
            'open_manipulator_sysid.excitation_trajectory.trajectory:main',
            'sysid_excitation_trajectory_plot = '
            'open_manipulator_sysid.excitation_trajectory.plot:main',
            'excitation_ga_optimize = '
            'open_manipulator_sysid.excitation_trajectory.ga_optimizer:main',
            'sysid_regress_synthetic = '
            'open_manipulator_sysid.system_identification.identify:main',
            'sysid_regress_bag = '
            'open_manipulator_sysid.system_identification.identify:main_bag',
            'sysid_identify_synthetic = '
            'open_manipulator_sysid.system_identification.identify:main',
            'sysid_identify_bag = '
            'open_manipulator_sysid.system_identification.identify:main_bag',
            'bag_to_dataset = '
            'open_manipulator_sysid.excitation_recording.bag_reader:main',
            'excitation_runner = '
            'open_manipulator_sysid.excitation_trajectory.runner:main',
            'sysid_bip_urdf_compare = '
            'open_manipulator_sysid.model_validation.bip_compare:main',
            'sysid_torque_compare_plot = '
            'open_manipulator_sysid.model_validation.torque_plot:main',
            'sysid_trajectory_compare_plot = '
            'open_manipulator_sysid.model_validation.trajectory_plot:main',
            'sysid_minimal_parameter_reference = '
            'open_manipulator_sysid.reference.minimal_parameters:main',
            'sysid_five_link_reference = '
            'open_manipulator_sysid.five_link_dynamics.reference_tables:main_reference',
            'sysid_five_link_identify_compare = '
            'open_manipulator_sysid.five_link_dynamics.reference_tables:main_identify_compare',
        ],
    },
)
