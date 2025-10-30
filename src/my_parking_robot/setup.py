from setuptools import find_packages, setup

package_name = 'my_parking_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_parking_robot.launch.xml']),
        ('share/' + package_name + '/rviz', ['rviz/my_parking_robot.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mgm',
    maintainer_email='doba.daniel@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_parking_robot = my_parking_robot.my_parking_robot:main",
            "control = my_parking_robot.control:main",
            "PathCreator = my_parking_robot.PathCreator:main"
        ],
    },
)
