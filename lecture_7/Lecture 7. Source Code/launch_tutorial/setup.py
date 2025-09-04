from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'launch_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='home',
    maintainer_email='home@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reverse_node = launch_tutorial.reverse_node:main',
            'random_velocity_node = launch_tutorial.random_velocity_node:main',
        ],
    },
)
