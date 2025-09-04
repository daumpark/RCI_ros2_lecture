from setuptools import find_packages, setup

package_name = 'topic_tutorial_py'

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
    maintainer='home',
    maintainer_email='home@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_vel_node = topic_tutorial_py.pubvel:main',
            'sub_pose_node = topic_tutorial_py.subpose:main',
            'direction_checker_node = topic_tutorial_py.direction:main',
        ],
    },
)
