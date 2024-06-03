import os
from glob import glob
from setuptools import setup

package_name = 'puzzlebot_ros'

gazebo_files = []
gazebo_files.append(('share/ament_index/resource_index/packages',['resource/' + package_name]))
gazebo_files.append(('share/' + package_name, ['package.xml']))
gazebo_files.append((os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')))
for dirpath, dirnames, filenames in os.walk('gazebo'):
   for filename in filenames:
       gazebo_files.append(('share/' + package_name+'/'+dirpath,[dirpath +'/'+filename]))


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=gazebo_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eduard Codres',
    maintainer_email='eduard.codres@manchester.ac.uk',
    description='Puzzlebot control examples',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
                'pwm_control = puzzlebot_ros.pwm_control:main',
                'velocity_control = puzzlebot_ros.velocity_control:main',
                'distance_control = puzzlebot_ros.distance_control:main',
                'kalman = puzzlebot_ros.kalman:main',
                'dead_reckoning = puzzlebot_ros.dead_reckoning:main',
                'goto_point = puzzlebot_ros.goto_point:main'
        ],
    },
)
