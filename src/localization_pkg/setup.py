from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'localization_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Adiciona uma regra para instalar a pasta 'resource' com seus arquivos
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan', # Mude para seu nome
    maintainer_email='ivan@todo.todo', # Mude para seu email
    description='Pacote de localização com filtro de partículas.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Cria o comando 'ros2 run localization_pkg localization_node'
            'localization_node = localization_pkg.LocalizationMain:main',
            # Cria o comando 'ros2 run localization_pkg simulation_node'
            'simulation_node = localization_pkg.Simulation:main',
            'teleop_key = localization_pkg.teleop_key:main',
        ],
    },
)
