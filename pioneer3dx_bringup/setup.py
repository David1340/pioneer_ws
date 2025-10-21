from setuptools import setup
import os

package_name = 'pioneer3dx_bringup'  # ROS 2 recomenda lowercase

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='davidnike80@gmail.com',
    description='Bringup do robô Pioneer 3DX',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pioneer3dx_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml'] if os.path.exists('config/params.yaml') else []),
    ],
)
