from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'states_routine'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aligning_body_routine = states_routine.aligning_body_routine:main',
            'walking_routine = states_routine.walking_routine:main',
            'kicking_routine = states_routine.kicking_routine:main',
            'idle_march_routine = states_routine.idle_march_routine:main',
            'idle_routine = states_routine.idle_routine:main',
            'getting_up_routine = states_routine.getting_up_routine:main',
            'searching_routine = states_routine.searching_routine:main'


        ],
    },
)
