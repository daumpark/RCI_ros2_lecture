from setuptools import setup
import os
from glob import glob

package_name = 'pantilt_action_tutorials_py'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fmrico',
    maintainer_email='jan.rosell@upc.edu',
    description='Package to illustrate actions',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pantilt_action_server = pantilt_action_tutorials_py.pantilt_action_server:main',
            'pantilt_action_client = pantilt_action_tutorials_py.pantilt_action_client:main',
        ],
    },
)