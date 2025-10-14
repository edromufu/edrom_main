import os
from glob import glob
from setuptools import setup

package_name = 'bhv_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.wbt'))),
        # Instala a pasta de texturas
        (os.path.join('share', package_name, 'worlds', 'textures'), glob(os.path.join('worlds', 'textures', '*.png'))),
        # Instala o nosso novo script controlador
        (os.path.join('share', package_name, 'controllers', 'bhv_sim'), glob(os.path.join('controllers', 'bhv_sim', 'bhv_sim'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vtr_caixeta',
    maintainer_email='vtr_caixeta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bhv_sim = bhv_simulator.bhv_sim:main',
        ],
    },
)