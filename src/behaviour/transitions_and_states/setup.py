from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'transitions_and_states'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vtr_caixeta',
    maintainer_email='victorvasconcelos676@gmail.com',
    description='Behaviour package — DSD integration',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'behaviour_node = transitions_and_states.behaviour_node:main',
            'dsd_node       = transitions_and_states.dsd_node:main',
        ],
    },
)