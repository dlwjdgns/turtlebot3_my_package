from setuptools import find_packages, setup

package_name = 'turtlebot3_my_package'

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
    maintainer='jhlee',
    maintainer_email='jhlee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtlebot3_my_ctrl = turtlebot3_my_package.turtlebot3_my_ctrl:main',
            'patrol_manager = turtlebot3_my_package.patrol_manager:main',
            'precision_docking_server = turtlebot3_my_package.precision_docking_server:main'
        ],
    },
)
