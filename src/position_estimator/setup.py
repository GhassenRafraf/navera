from setuptools import find_packages, setup

package_name = 'position_estimator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghassen',
    maintainer_email='grafraf08@gmail.com',
    description='Position Estimator for Drones',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'ukf_node = position_estimator.ukf_node:main',
		'celestial_vision_node = position_estimator.celestial_vision_node:main',
		'aerial_vision_node = position_estimator.aerial_vision_node:main',
		'GPS_node = position_estimator.GPS_node:main',
		'IMU_node = position_estimator.IMU_node:main',
		'gpsDriverNode = position_estimator.gpsDriverNode:main',
		'imuDriverNode = position_estimator.imuDriverNode:main',
		'aerialAdjustmentNode = position_estimator.aerial_adjustment_node:main',
		'celestialAdjustmentNode = position_estimator.celestial_adjustment_node:main',
        ],
    },
)
