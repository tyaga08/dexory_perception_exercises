from setuptools import setup

package_name = 'lidar_data_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tyagaraja Ramaswamy',
    maintainer_email='rtyagaraja@gmail.com',
    description='Package to analyze the LiDAR data problem from Dexory',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_analysis_node = lidar_data_analysis.sensor_analysis_node:main'
        ],
    },
)
