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
            'planning = trajcontrol.planning:main',
            'estimator = trajcontrol.estimator:main',
            'controller_manual_smart = trajcontrol.controller_manual_smart:main',
            'controller_mpc = trajcontrol.controller_mpc:main',
            'save_file = trajcontrol.save_file:main',
            'benchmark2 = trajcontrol.benchmark2:main',
            'virtual_smart_needle = trajcontrol.virtual_smart_needle:main',
        ],
    },
)
