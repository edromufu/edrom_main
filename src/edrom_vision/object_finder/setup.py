from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_finder'
__version__ = '0.0.1'

setup(
    name=package_name,
    version=__version__,
    
    packages=find_packages(exclude=['docs', 'tests*']),
    
    # data_files informa ao colcon quais arquivos adicionais instalar.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Instala o nosso launch file unificado
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # Instala o modelo .pt para a simulação
        (os.path.join('share', package_name, 'modelo'), glob('modelo/*.pt') + glob('modelo/*.onnx')),
        (os.path.join('share', package_name), glob('resource/*')),
    ],
    
    # install_requires lista as dependências de Python que o pip deve instalar.
    install_requires=['setuptools', 'opencv-python', 'ultralytics'],
    
    zip_safe=True,
    description='Pacote de detecção de objetos para a EDROM',
    long_description='Este programa detecta bola, robôs e outros elementos do campo de futebol de robôs.',
    license='BSD',
    
    entry_points={
        'console_scripts': [
            # Certifique-se de que o nome do arquivo aqui é o nome do seu script unificado final.
            'finder = object_finder.connecting_and_showing:main',
        ],
    },
    
    author='EDROM',
)