from setuptools import find_packages, setup

package_name = 'hello_ros_py'

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
    maintainer='Sanghyun Kim',
    maintainer_email='kim87@khu.ac.kr',
    description='Send hello world',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world_node=hello_ros_py.practice2:main',
            'logging_node=hello_ros_py.practice3:main',
        ],
    },
)
