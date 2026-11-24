from setuptools import find_packages, setup

package_name = 'imu_ros_arduino'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/imu_filters.yaml']),
        ('share/' + package_name + '/launch', ['launch/imu_filters.launch.py',
                                                  'launch/ipm_filter_print.launch.py']),
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
            'imu_read = imu_ros_arduino.imu_read:main',
            'imu_filters = imu_ros_arduino.imu_filters_node:main',
            'ipm_filter_eval = imu_ros_arduino.ipm_eval:main',
            'ipm_filter_print = imu_ros_arduino.ipm_filter_print:main',
        ],
    },
)
