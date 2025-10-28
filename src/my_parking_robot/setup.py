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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrisgerber',
    maintainer_email='andrisgerber2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test_node = my_parking_robot.my_first_node:main",
            "draw_circle = my_parking_robot.draw_circle:main",
            "pose_subscriber = my_parking_robot.pose_subscriber:main",
            "turtle_controller = my_parking_robot.turtle_controller:main",
            "andras_handwritten = my_parking_robot.andras_handwritten:main"
        
        ],
    },
)