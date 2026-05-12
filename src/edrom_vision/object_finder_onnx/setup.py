from setuptools import setup
from glob import glob
import os

package_name = 'object_finder_onnx'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'modelo'), glob('modelo/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EDROM',
    maintainer_email='edromufu@gmail.com',
    description='Pacote de teste ONNX',
    license='MIT',
    entry_points={
        'console_scripts': [
            'finder_onnx = object_finder_onnx.finder_onnx:main',
        ],
    },
)
