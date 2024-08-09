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
		'gps_ecef = position_estimator.gps_ecef:main',
		'ecef_gps = position_estimator.ecef_gps:main',
        ],
    },
)
