from setuptools import find_packages, setup

package_name = 'aurea_webots'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/aurea_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/aurea_motion.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/aurea_urdf_pkg.urdf']))
data_files.append(('share/' + package_name + '/controllers', ['controllers/aurea_driver/aurea_driver.py']))
data_files.append(('share/' + package_name + '/protos', ['protos/Aurea.proto']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
	data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedrohperes',
    maintainer_email='pedrohperescode@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aurea_sensors = aurea_webots.aurea_sensors:main'
        ],
    },
)
