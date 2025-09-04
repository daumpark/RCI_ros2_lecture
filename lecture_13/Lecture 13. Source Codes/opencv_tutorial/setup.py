from setuptools import setup

package_name = 'opencv_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/opencv_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_node = opencv_tutorial.opencv_tutorial:main',
            'aruco_tf_node = opencv_tutorial.aruco_tf_node:main',
            'gpt_node = opencv_tutorial.gpt_node:main',            
        ],
    },
)
