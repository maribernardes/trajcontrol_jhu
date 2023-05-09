import os
from glob import glob
from setuptools import setup

package_name = 'trajcontrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mariana Bernardes',
    maintainer_email='bernardes@unb.br',
    description='Needle trajectory compensation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keypress = trajcontrol.keypress:main',
            'depth_measurement = trajcontrol.depth_measurement:main',
            'sensor_processing = trajcontrol.sensor_processing:main',
            'sensor_processing_step = trajcontrol.sensor_processing_step:main',
            'estimator = trajcontrol.estimator:main',
            'controller_init = trajcontrol.controller_init:main',
            'controller_manual = trajcontrol.controller_manual:main',
            'controller_sequence = trajcontrol.controller_sequence:main',
            'controller_rand = trajcontrol.controller_rand:main',
            'controller_proportional = trajcontrol.controller_proportional:main',
            'controller_mpc = trajcontrol.controller_mpc:main',
            'registration = trajcontrol.registration:main',
            'save_file = trajcontrol.save_file:main',
            'trajcontrol_demo = trajcontrol.trajcontrol_demo:main',
            'trajcontrol_demo_step = trajcontrol.trajcontrol_demo_step:main',
        ],
    },
)
