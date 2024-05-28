from setuptools import find_packages, setup
import os 

package_name = 'my_robot_test'

config_path = 'config'
config_files = [os.path.join('config', file) for file in os.listdir(config_path)] if os.path.exists(config_path) else []

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkyu',
    maintainer_email='dknjy3313@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_robot = my_robot_test.navigate_robot:main',
            'emergency_bracking = my_robot_test.emergency_bracking:main',
            'collision_avoidance = my_robot_test.collision_avoidance:main',
            'obstacle_avoidance = my_robot_test.obstacle_avoidance:main',
        ],
    },
)
